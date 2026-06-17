import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CSICameraNode(Node):
    def __init__(self):
        super().__init__('csi_camera_node')
        
        # 이전 ArUco 스크립트가 구독할 /image_raw 토픽 발행
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 FPS
        self.cv_bridge = CvBridge()
        
        # 젯슨 나노 CSI 카메라 전용 GStreamer 파이프라인
        gstreamer_pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)30/1 ! "
            "nvvidconv flip-method=2 ! "  #카메라 180도 반전
            "video/x-raw, width=640, height=480, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
        )
        
        self.get_logger().info('카메라 연결을 시도합니다...')
        self.cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다. 물리적 연결을 확인하세요.')
        else:
            self.get_logger().info('CSI 카메라 노드가 정상적으로 실행되었습니다.')

    def timer_callback(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # OpenCV 이미지를 ROS2 Image 메시지로 변환하여 발행
                msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher_.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CSICameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
