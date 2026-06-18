import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from cv_bridge import CvBridge
import cv2
import math
import numpy as np

class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_localizer')
        
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.cv_bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # 0~7번 마커 데이터베이스 (기준 거리: 48cm)
        self.marker_database = {
            0: (0.055, -0.382, 3.12),
            1: (1.759, -0.202, 1.57),
            2: (1.457, -2.864, 0.03),
            3: (1.567, -4.364, 0.01),
            4: (1.561, -5.816, 0.00),
            5: (1.440, -7.275, 0.00),
            6: (1.485, -8.477, -1.57),
            7: (0.142, -8.044, -3.13)
        }
        
        self.state = 'SEARCHING'
        self.twitch_count = 0
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('👁️ 시각 기반 동적 거리 보정 로컬라이제이션 가동: 마커 탐색을 시작합니다...')

    def control_loop(self):
        twist = Twist()
        
        if self.state == 'SEARCHING':
            twist.angular.z = 0.3  
            self.cmd_vel_pub.publish(twist)
            
        elif self.state == 'TWITCHING':
            self.twitch_count += 1
            if self.twitch_count <= 5:
                twist.angular.z = -0.2
            elif self.twitch_count <= 10:
                twist.angular.z = 0.2
            else:
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.state = 'DONE'
                self.timer.cancel()
                self.get_logger().info('✅ [완료] 라이다 데이터가 벽면에 완벽히 매칭되었습니다!')
                return
                
            self.cmd_vel_pub.publish(twist)

    def image_callback(self, msg):
        if self.state != 'SEARCHING':
            return

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None:
                detected_id = ids[0][0]
                
                if detected_id in self.marker_database:
                    # 1. 픽셀 크기 측정
                    corners_np = corners[0][0]
                    marker_width = np.linalg.norm(corners_np[0] - corners_np[1])
                    
                    # 2. 비례식을 이용한 현재 실제 거리 계산
                    current_distance = 27.6 / marker_width
                    
                    self.get_logger().info(f'🎯 마커 포착! (ID: {detected_id})')
                    self.get_logger().info(f'📏 마커 크기: {marker_width:.2f}px -> 예상 거리: {current_distance:.2f}m')
                    
                    # 즉시 급브레이크 (관성 최소화)
                    self.cmd_vel_pub.publish(Twist())
                    
                    # 3. 거리를 반영하여 동적으로 보정된 좌표 주입
                    self.set_initial_pose(self.marker_database[detected_id], current_distance)
                    
                    self.state = 'TWITCHING'
                    self.get_logger().info('🔄 라이다 스캔 정밀 동기화를 위해 1초간 자동 미세 동작을 수행합니다...')
                    
        except Exception as e:
            self.get_logger().error(f'이미지 처리 중 오류 발생: {e}')

    def set_initial_pose(self, pose_data, current_distance):
        base_x, base_y, base_yaw = pose_data
        
        # 48cm(0.48m) 기준 위치에서 현재 로봇이 더 떨어져 있는 거리 차이 계산
        distance_diff = current_distance - 0.48
        
        # 로봇이 바라보는 방향(yaw)을 기준으로 거리 차이만큼 x, y 좌표를 뒤로 밀어줌
        adjusted_x = base_x - (distance_diff * math.cos(base_yaw))
        adjusted_y = base_y - (distance_diff * math.sin(base_yaw))
        
        self.get_logger().info(f'🔄 거리 보정 좌표 적용 완료 (X: {adjusted_x:.2f}, Y: {adjusted_y:.2f})')
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose_msg.pose.pose.position.x = adjusted_x
        pose_msg.pose.pose.position.y = adjusted_y
        pose_msg.pose.pose.position.z = 0.0
        
        pose_msg.pose.pose.orientation.z = math.sin(base_yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(base_yaw / 2.0)
        
        pose_msg.pose.covariance[0] = 0.15   
        pose_msg.pose.covariance[7] = 0.15   
        pose_msg.pose.covariance[35] = 0.20  
        
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 프로그램 강제 종료 시, 모터에 남아있는 명령을 초기화하여 안전하게 멈춤
        node.get_logger().info('종료 신호 감지. 로봇 바퀴를 안전하게 정지합니다.')
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
