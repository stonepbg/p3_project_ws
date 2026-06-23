import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Int32, String
from std_srvs.srv import Empty
import sys
import time

class AmclAutoLocalizer(Node):
    def __init__(self):
        super().__init__('amcl_auto_localizer')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Int32, '/AGV_status', 10)
        self.log_pub = self.create_publisher(String, '/AGV_log', 10)
        
        # AMCL이 추정하는 현재 위치와 불확실성(Covariance)을 구독
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        
        # 글로벌 로컬라이제이션 서비스 클라이언트
        self.global_loc_client = self.create_client(Empty, '/reinitialize_global_localization')
        
        self.state = 'INIT'
        self.covariance_threshold = 0.05  # 위치 신뢰도 기준치 (낮을수록 더 깐깐하게 판단)
        self.start_time = time.time()
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('🧭 순수 라이다(AMCL) 기반 로컬라이제이션 가동')

    def send_log(self, text, level='info'):
        if level == 'info': self.get_logger().info(text)
        elif level == 'warn': self.get_logger().warn(text)
        
        msg = String()
        msg.data = f"[로컬라이제이션] {text}"
        self.log_pub.publish(msg)

    def control_loop(self):
        twist = Twist()
        current_time = time.time()
        
        if self.state == 'INIT':
            # 서비스가 준비되었는지 확인 후 맵 전체에 파티클 분포
            if self.global_loc_client.wait_for_service(timeout_sec=1.0):
                req = Empty.Request()
                self.global_loc_client.call_async(req)
                self.send_log('🌍 맵 전체에 파티클을 뿌렸습니다. 수렴을 위한 회전을 시작합니다.')
                self.state = 'SPINNING'
                self.start_time = current_time
            else:
                self.send_log('서비스 대기 중... (/reinitialize_global_localization)', 'warn')
                
        elif self.state == 'SPINNING':
            # 제자리 회전하며 라이다 스캔 매칭 유도
            twist.angular.z = 0.3
            self.cmd_vel_pub.publish(twist)
            
            # 60초 이상 수렴하지 못하면 경고 출력
            if current_time - self.start_time > 60.0:
                self.send_log('⚠️ 파티클 수렴 지연. 맵의 대칭성이 강하거나 특징이 부족할 수 있습니다.', 'warn')
                self.start_time = current_time # 타이머 리셋 후 계속 시도

    def pose_callback(self, msg):
        if self.state != 'SPINNING': return
        
        # 공분산(Covariance) 행렬에서 x, y, yaw의 불확실성 분산 값 추출
        cov = msg.pose.covariance
        var_x = cov[0]
        var_y = cov[7]
        var_yaw = cov[35]
        
        # 분산 값이 기준치 이하로 떨어지면 파티클이 한 곳으로 뭉쳤다(수렴했다)고 판단
        if var_x < self.covariance_threshold and var_y < self.covariance_threshold and var_yaw < (self.covariance_threshold * 2):
            self.state = 'DONE'
            self.cmd_vel_pub.publish(Twist()) # 정지
            
            self.send_log(f'✅ 위치 파악 완료! (오차 분산 - x:{var_x:.3f}, y:{var_y:.3f}, yaw:{var_yaw:.3f})')
            
            status_msg = Int32()
            status_msg.data = 0
            self.status_pub.publish(status_msg)
            
            time.sleep(1.0)
            sys.exit(0) # 기존처럼 노드 종료 시 Launch에 의해 시스템이 다음 단계로 넘어가도록 처리

def main(args=None):
    rclpy.init(args=args)
    node = AmclAutoLocalizer()
    try: rclpy.spin(node)
    except SystemExit: pass
    except KeyboardInterrupt: node.cmd_vel_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
