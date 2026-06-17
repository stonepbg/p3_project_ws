import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

class AutoLocalizer(Node):
    def __init__(self):
        super().__init__('auto_localizer')

        self.srv_client = self.create_client(Empty, '/reinitialize_global_localization')
        while not self.srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기 중... (/reinitialize_global_localization)')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            self.pose_callback, 
            10
        )

        self.is_converged = False
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.move_robot)

        # 이동 패턴 제어를 위한 변수
        self.tick_count = 0
        self.state = 'ROTATE' # 'ROTATE' 또는 'EXPLORE' 상태

        self.call_global_localization()

    def call_global_localization(self):
        self.get_logger().info('1단계: 맵 전체에 파티클을 분산시킵니다.')
        req = Empty.Request()
        self.srv_client.call_async(req)

    def move_robot(self):
        if self.is_converged:
            return

        self.tick_count += 1
        twist = Twist()

        # 2단계: 맵의 대칭성을 벗어나기 위해 제자리 회전과 이동을 번갈아 수행합니다.
        # 5초(10 ticks) 회전 후, 3초(6 ticks) 곡선 주행 반복
        if self.state == 'ROTATE':
            twist.angular.z = 0.5
            if self.tick_count >= 10:
                self.state = 'EXPLORE'
                self.tick_count = 0
        elif self.state == 'EXPLORE':
            twist.linear.x = 0.1  # 천천히 직진
            twist.angular.z = 0.2 # 약간 회전을 주어 호를 그리며 주행 (충돌 방지 및 스캔 범위 확대)
            if self.tick_count >= 6:
                self.state = 'ROTATE'
                self.tick_count = 0
        
        self.cmd_vel_pub.publish(twist)

    def pose_callback(self, msg):
        if self.is_converged:
            return

        cov_x = msg.pose.covariance[0]
        cov_y = msg.pose.covariance[7]
        cov_yaw = msg.pose.covariance[35]

        # 로그 도배를 막기 위해 1초(2 ticks)마다 한 번씩만 출력
        if self.tick_count % 2 == 0:
            self.get_logger().info(f'[상태: {self.state}] 불확실성 -> x: {cov_x:.4f}, y: {cov_y:.4f}, yaw: {cov_yaw:.4f}')

        # 3단계: 임계값(Threshold)을 0.05에서 0.015로 대폭 강화 (오탐지 방지)
        threshold = 0.015 

        if cov_x < threshold and cov_y < threshold and cov_yaw < threshold:
            self.is_converged = True
            self.get_logger().info('★★★ 3단계 완료: 위치 파악 완료! 파티클이 정확히 수렴되었습니다. ★★★')

            # 로봇 정지
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = AutoLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
