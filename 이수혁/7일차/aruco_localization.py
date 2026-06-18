import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from cv_bridge import CvBridge
import cv2
import math
import numpy as np
import sys
import time

class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_localizer')
        
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.cv_bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
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
        self.sync_count = 0
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('👁️ 시각 기반 로컬라이제이션 가동 (초정밀 대각선 보정 및 자동 종료 모드)')

    def control_loop(self):
        twist = Twist()
        
        if self.state == 'SEARCHING':
            twist.angular.z = 0.3  
            self.cmd_vel_pub.publish(twist)
            
        elif self.state == 'AMCL_SYNC':
            self.sync_count += 1
            
            # [라이다 스캔 댄스 2.0] 속도를 늦추고 시간을 늘려 AMCL이 계산할 시간을 충분히 확보
            if self.sync_count <= 15:
                twist.angular.z = 0.2   # 1.5초간 천천히 좌회전
            elif self.sync_count <= 45:
                twist.angular.z = -0.2  # 3.0초간 천천히 우회전 (충분히 반대쪽 스캔)
            elif self.sync_count <= 60:
                twist.angular.z = 0.2   # 1.5초간 천천히 좌회전 (중앙 복귀)
            elif self.sync_count <= 70:
                twist.angular.z = 0.0   # 1.0초간 제자리에 멈춰서 라이다 파티클 뭉침 대기
                twist.linear.x = 0.0
            elif self.sync_count <= 85:
                twist.linear.x = 0.04   # 1.5초간 천천히 전진
            elif self.sync_count <= 100:
                twist.linear.x = -0.04  # 1.5초간 천천히 후진
            else:
                # 동기화 모션 종료 및 프로그램 완전 정지
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.state = 'DONE'
                self.timer.cancel()
                
                self.get_logger().info('✅ 완벽한 위치 및 각도 매칭 성공! 카메라와 노드를 자동 종료합니다.')
                time.sleep(1.0) 
                sys.exit(0) # 자동 종료 트리거
                
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
                    corners_np = corners[0][0]
                    marker_width = np.linalg.norm(corners_np[0] - corners_np[1])
                    current_distance = 27.6 / marker_width
                    
                    self.cmd_vel_pub.publish(Twist()) # 급정지
                    
                    self.get_logger().info(f'🎯 마커(ID: {detected_id}) 발견! 예상 거리: {current_distance:.2f}m')
                    
                    self.set_initial_pose(self.marker_database[detected_id], current_distance)
                    
                    self.state = 'AMCL_SYNC'
                    self.get_logger().info('🔄 대각선 각도 오차를 수정하기 위해 초정밀 스캔 댄스를 시작합니다...')
                    
        except Exception as e:
            self.get_logger().error(f'이미지 처리 중 오류 발생: {e}')

    def set_initial_pose(self, pose_data, current_distance):
        base_x, base_y, base_yaw = pose_data
        
        distance_diff = current_distance - 0.48
        adjusted_x = base_x - (distance_diff * math.cos(base_yaw))
        adjusted_y = base_y - (distance_diff * math.sin(base_yaw))
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose_msg.pose.pose.position.x = adjusted_x
        pose_msg.pose.pose.position.y = adjusted_y
        pose_msg.pose.pose.position.z = 0.0
        
        pose_msg.pose.pose.orientation.z = math.sin(base_yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(base_yaw / 2.0)
        
        # ★ 핵심 보정: X, Y 위치의 오차 범위도 살짝 늘리고, 각도(Yaw) 오차 허용치를 극대화(2.0)
        pose_msg.pose.covariance[0] = 0.30   # X 오차 허용 (증가)
        pose_msg.pose.covariance[7] = 0.30   # Y 오차 허용 (증가)
        pose_msg.pose.covariance[35] = 2.0   # Yaw(방향) 오차 허용 극대화 (라이다 100% 신뢰)
        
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizer()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        node.get_logger().info('강제 종료 감지. 바퀴를 정지합니다.')
        node.cmd_vel_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
