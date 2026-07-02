#!/usr/bin/env python3
import os
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage  
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import time

# ==============================================================================
# 환경 설정: ROS_DOMAIN_ID 지정 (rclpy.init 전 실행 필수)
# ==============================================================================
os.environ['ROS_DOMAIN_ID'] = '30'

class StereoLaserBalancedTrackingNode(Node):

    def __init__(self):
        super().__init__('stereo_laser_balanced_tracking_node')

        # QoS 및 퍼블리셔 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.compressed_publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', qos_profile)
        
        # WSL2 내 V4L2 장치 초기화
        DEVICE_ID = 0
        self.get_logger().info(f'WIT 스테레오 카메라(/dev/video{DEVICE_ID}) 원거리 밸런스 필터 모드를 시작합니다.')
        
        self.cap = cv2.VideoCapture(DEVICE_ID, cv2.CAP_V4L2)
        
        # 하드웨어 Side-by-Side 1280x480 및 15 FPS 설정
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        time.sleep(1.0)

        if not self.cap.isOpened():
            self.get_logger().error('카메라 장치를 열 수 없습니다!')
            return

        # 직접 픽셀 추적 기반 거리 측정 파라미터 (630.0 정밀 세팅 반영)
        self.FOCAL_LENGTH = 630.0  
        self.BASELINE = 0.06       

        # 백그라운드 프레임 캡처 스레드 레이어
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.last_valid_frame = None 
        self.is_running = True
        
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        # 15 FPS 주기로 연산 타이머 구동
        self.timer = self.create_timer(1.0 / 15.0, self.timer_callback)
        self.get_logger().info('원거리 가시성 확보 및 오검출 밸런스 시스템 가동')
        
    def _capture_loop(self):
        while self.is_running:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.frame_lock:
                    self.latest_frame = frame
            else:
                time.sleep(0.01)
            time.sleep(0.001)

    def is_frame_corrupted(self, img):
        """회색 잡화면 방지 필터"""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, stddev = cv2.meanStdDev(gray)
        if stddev[0][0] < 12.0:
            return True
        return False

    def find_laser_pointer(self, img):
        """
        [원거리 하이브리드 필터 적용]
        멀리 있어 크기가 작고 명도가 조금 낮아진 레이저(Value >= 215)도 잡을 수 있도록 완화하되,
        채도(Saturation) 기준을 높여 노이즈를 방어합니다.
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # 명도(Value) 하한선을 240에서 215로 낮추어 멀리 있는 레이저의 감도를 확보
        # 대신 채도(Saturation) 하한선을 30에서 80으로 높여 붉은색이 아닌 일반 밝은 노이즈(살색, 형광등) 제거
        lower_red1 = np.array([0, 80, 215])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 80, 215])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # 원거리 소형 레이저 유실을 막기 위해 침식(erode) 후 팽창(dilate) 연산 밸런스 조절
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            # 원거리의 작은 점(2픽셀 이상)도 레이저 포인터 후보군으로 인정하도록 수정
            if cv2.contourArea(largest_contour) >= 2: 
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    return (cX, cY)
        return None

    def timer_callback(self):
        raw_frame = None
        with self.frame_lock:
            if self.latest_frame is not None:
                raw_frame = self.latest_frame.copy()
        
        if raw_frame is not None:
            h, w, _ = raw_frame.shape
            if w != 1280 or h != 480:
                raw_frame = cv2.resize(raw_frame, (1280, 480))

            if self.is_frame_corrupted(raw_frame):
                if self.last_valid_frame is not None:
                    frame = self.last_valid_frame.copy()
                else:
                    return
            else:
                self.last_valid_frame = raw_frame.copy()
                frame = raw_frame

            # 좌안(Left)과 우안(Right) 프레임 분할 크롭
            left_img = frame[0:480, 0:640]
            right_img = frame[0:480, 640:1280]

            display_left = left_img.copy()
            display_right = right_img.copy()

            # 양쪽 카메라에서 각각 독립적으로 레이저 검출
            left_laser = self.find_laser_pointer(left_img)
            right_laser = self.find_laser_pointer(right_img)

            if right_laser is not None:
                cv2.circle(display_right, (right_laser[0], right_laser[1]), 4, (0, 0, 255), -1)

            # 양쪽 모두 검출된 상태에서 정확한 거리 계산
            if left_laser is not None and right_laser is not None:
                lx, ly = left_laser
                rx, ry = right_laser
                
                disp_val = float(lx - rx)
                
                if disp_val > 0.0:
                    depth_m = (self.FOCAL_LENGTH * self.BASELINE) / disp_val
                    
                    box_size = 20
                    pt1 = (max(0, lx - box_size), max(0, ly - box_size))
                    pt2 = (min(640, lx + box_size), min(480, ly + box_size))
                    
                    cv2.rectangle(display_left, pt1, pt2, (0, 255, 0), 2)
                    cv2.circle(display_left, (lx, ly), 3, (0, 0, 255), -1)
                    
                    text_pos = (max(0, lx - box_size), max(25, ly - box_size - 5))
                    cv2.putText(display_left, f"Depth: {depth_m:.2f}m", text_pos,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                else:
                    cv2.circle(display_left, (lx, ly), 4, (0, 0, 255), -1)
                    cv2.putText(display_left, "Calc Error (Disp <= 0)", (lx - 30, ly - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            elif left_laser is not None:
                # 한쪽 눈만 보일 때 정보 마킹 유지
                lx, ly = left_laser
                cv2.circle(display_left, (lx, ly), 4, (0, 0, 255), -1)
                cv2.putText(display_left, "Searching Right Laser...", (lx - 40, ly - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # 모니터 가로 병합
            combined_monitor = np.hstack((display_left, display_right))
            cv2.line(combined_monitor, (640, 0), (640, 480), (255, 0, 0), 2)
            
            cv2.putText(combined_monitor, "MAIN (Left - Balanced Mode)", (20, 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(combined_monitor, "SUB (Right Eye)", (660, 35), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow("Stereo Dual Video Monitor (Laser Depth Mode)", combined_monitor)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('사용자 요청으로 모니터링을 종료합니다.')

            # ROS 2 이미지 퍼블리시
            comp_msg = CompressedImage()
            comp_msg.header.stamp = self.get_clock().now().to_msg()
            comp_msg.header.frame_id = "camera_stereo_link"
            comp_msg.format = "jpeg"
            
            _, compressed_img = cv2.imencode('.jpg', combined_monitor, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
            comp_msg.data = compressed_img.tobytes()
            self.compressed_publisher_.publish(comp_msg)
        else:
            self.get_logger().debug('카메라 버퍼 확보 실패로 인해 레이저 추적을 스킵합니다.')

def main(args=None):
    rclpy.init(args=args)
    node = StereoLaserBalancedTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.is_running = False
        if hasattr(node, 'cap') and node.cap.isOpened():
            node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()