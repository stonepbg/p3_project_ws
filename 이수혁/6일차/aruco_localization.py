import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import math

class ArucoLocalizer(Node):
    def __init__(self):
        super().__init__('aruco_localizer')
        
        # 카메라 이미지 구독
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # 실제 카메라 토픽 이름에 맞게 수정 필요
            self.image_callback,
            10)
            
        # 초기 위치(Initial Pose) 퍼블리셔
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.cv_bridge = CvBridge()
        
        # OpenCV ArUco 딕셔너리 설정 (DICT_6X6_250 사용 가정)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        # 마커 ID별 맵 상의 실제 좌표 (x, y, yaw) 
        # 사용자가 마커를 붙인 위치 앞의 로봇 좌표를 미리 측정해서 적어둡니다.
        self.marker_database = {
            0: (1.5, 2.0, 0.0),   # 예: 0번 마커를 보면 (1.5, 2.0) 위치에 0도(정면)를 보고 있다고 확정
            1: (4.0, -1.0, 1.57)  # 예: 1번 마커 좌표
        }
        
        self.is_localized = False
        self.get_logger().info('ArUco 기반 위치 초기화 노드가 실행되었습니다. 카메라에서 마커를 찾습니다...')

    def image_callback(self, msg):
        if self.is_localized:
            return

        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # ArUco 마커 감지
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None:
                detected_id = ids[0][0]
                self.get_logger().info(f'마커 감지됨! ID: {detected_id}')
                
                if detected_id in self.marker_database:
                    self.set_initial_pose(self.marker_database[detected_id])
                    self.is_localized = True
                    self.get_logger().info('위치 초기화 성공. 노드 동작을 정지합니다.')
                else:
                    self.get_logger().info('등록되지 않은 마커 ID입니다.')
                    
        except Exception as e:
            self.get_logger().error(f'이미지 처리 중 오류 발생: {e}')

    def set_initial_pose(self, pose_data):
        x, y, yaw = pose_data
        
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 위치 입력
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        # Yaw 각도를 쿼터니언(Quaternion)으로 변환
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 공분산(확신도) 설정 - 매우 확신함(값이 작음)으로 설정하여 파티클을 좁힘
        pose_msg.pose.covariance[0] = 0.05   # x
        pose_msg.pose.covariance[7] = 0.05   # y
        pose_msg.pose.covariance[35] = 0.05  # yaw
        
        self.pose_pub.publish(pose_msg)
        self.get_logger().info(f'/initialpose 토픽 발행 완료: x={x}, y={y}, yaw={yaw}')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
