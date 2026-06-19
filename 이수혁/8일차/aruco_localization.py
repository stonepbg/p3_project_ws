import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Int32, String
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
        self.status_pub = self.create_publisher(Int32, '/AGV_status', 10)
        
        # [신규] 대시보드 퍼블리셔
        self.log_pub = self.create_publisher(String, '/AGV_log', 10)
        
        self.cv_bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        self.marker_database = {
            0: (0.055, -0.382, 3.12), 1: (1.759, -0.202, 1.57),
            2: (1.457, -2.864, 0.03), 3: (1.567, -4.364, 0.01),
            4: (1.561, -5.816, 0.00), 5: (1.440, -7.275, 0.00),
            6: (1.485, -8.477, -1.57), 7: (0.142, -8.044, -3.13)
        }
        
        self.state = 'SEARCHING'
        self.sync_count = 0
        self.search_start_time = time.time()
        self.shift_start_time = 0.0
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('👁️ 시각 기반 로컬라이제이션 가동')

    def send_log(self, text, level='info'):
        if level == 'info': self.get_logger().info(text)
        elif level == 'warn': self.get_logger().warn(text)
        
        msg = String()
        msg.data = f"[비전] {text}"
        self.log_pub.publish(msg)

    def control_loop(self):
        twist = Twist()
        current_time = time.time()
        
        if self.state == 'SEARCHING':
            if current_time - self.search_start_time > 25.0:
                self.send_log('⚠️ 1바퀴 회전 완료. 사각지대 탈출을 위해 위치를 이동합니다.', 'warn')
                self.state = 'SHIFTING'
                self.shift_start_time = current_time
            else:
                twist.angular.z = 0.3  
                self.cmd_vel_pub.publish(twist)
                
        elif self.state == 'SHIFTING':
            if current_time - self.shift_start_time < 3.0:
                twist.linear.x = 0.15
            else:
                self.send_log('🔄 위치 이동 완료. 다시 주변 탐색을 시작합니다.')
                self.state = 'SEARCHING'
                self.search_start_time = time.time() 
            self.cmd_vel_pub.publish(twist)
            
        elif self.state == 'AMCL_SYNC':
            self.sync_count += 1
            if self.sync_count <= 15: twist.angular.z = 0.2   
            elif self.sync_count <= 45: twist.angular.z = -0.2  
            elif self.sync_count <= 60: twist.angular.z = 0.2   
            elif self.sync_count <= 70: twist.angular.z = 0.0; twist.linear.x = 0.0
            elif self.sync_count <= 85: twist.linear.x = 0.04   
            elif self.sync_count <= 100: twist.linear.x = -0.04  
            else:
                self.cmd_vel_pub.publish(Twist())
                self.state = 'DONE'
                self.timer.cancel()
                
                self.send_log('✅ 완벽한 위치 및 각도 매칭 성공! 자율주행을 준비합니다.')
                
                status_msg = Int32()
                status_msg.data = 0
                self.status_pub.publish(status_msg)
                
                time.sleep(1.0) 
                sys.exit(0) 
            self.cmd_vel_pub.publish(twist)

    def image_callback(self, msg):
        if self.state not in ['SEARCHING', 'SHIFTING']: return
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None:
                detected_id = ids[0][0]
                if detected_id in self.marker_database:
                    self.cmd_vel_pub.publish(Twist())
                    
                    corners_np = corners[0][0]
                    marker_width = np.linalg.norm(corners_np[0] - corners_np[1])
                    current_distance = 27.6 / marker_width
                    
                    self.send_log(f'🎯 마커(ID: {detected_id}) 발견! 동기화 스캔 댄스를 시작합니다.')
                    self.set_initial_pose(self.marker_database[detected_id], current_distance)
                    self.state = 'AMCL_SYNC' 
        except Exception: pass

    def set_initial_pose(self, pose_data, current_distance):
        base_x, base_y, base_yaw = pose_data
        distance_diff = current_distance - 0.48
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = base_x - (distance_diff * math.cos(base_yaw))
        pose_msg.pose.pose.position.y = base_y - (distance_diff * math.sin(base_yaw))
        pose_msg.pose.pose.orientation.z = math.sin(base_yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(base_yaw / 2.0)
        pose_msg.pose.covariance[0] = 0.30   
        pose_msg.pose.covariance[7] = 0.30   
        pose_msg.pose.covariance[35] = 2.0   
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizer()
    try: rclpy.spin(node)
    except SystemExit: pass 
    except KeyboardInterrupt: node.cmd_vel_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()