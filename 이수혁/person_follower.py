import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
import time

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        # 1. Publisher & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_sub = self.create_subscription(Twist, '/target_cmd', self.target_callback, 10)
        self.search_sub = self.create_subscription(String, '/search_cmd', self.search_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # 2. 로봇 제어 파라미터 (환경에 맞게 수정 가능)
        self.target_distance = 1.0      # 사람과 유지할 목표 간격 (1.0m)
        self.safe_distance = 0.35       # [신규] 벽 충돌 방지 안전 거리 (0.35m)
        self.max_linear_vel = 0.4       # 최대 전진/후진 속도 (m/s)
        self.max_angular_vel = 0.5      # 사람 추종 시 최대 회전 속도 (rad/s)
        self.search_angular_vel = 0.4   # [신규] 사람 탐색 시 제자리 회전 속도 (rad/s)
        self.watchdog_timeout = 0.5     # [신규] 통신 두절 시 정지하는 타임아웃 (0.5초)

        # 3. 상태 저장 변수들
        self.last_target_time = 0.0
        self.last_search_time = 0.0
        self.target_dist = 0.0
        self.target_angle = 0.0
        self.search_dir = ""
        self.min_dist_front = float('inf')
        self.min_dist_back = float('inf')

        # 4. 메인 제어 루프 (1초에 20번 실행)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Person Follower Node (LiDAR Safety & Search Mode) Started!')

    # --- 콜백 함수들 ---
    def target_callback(self, msg):
        self.target_dist = msg.linear.x
        self.target_angle = msg.angular.z
        self.last_target_time = time.time()

    def search_callback(self, msg):
        self.search_dir = msg.data
        self.last_search_time = time.time()

    def scan_callback(self, msg):
        min_f = float('inf')
        min_b = float('inf')

        for i, r in enumerate(msg.ranges):
            # 노이즈 및 측정 불가 데이터 필터링
            if r < 0.05 or r > 10.0 or math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            deg = math.degrees(angle)

            # 전방 시야각 (-30도 ~ 30도) 최소 거리
            if -30 <= deg <= 30:
                min_f = min(min_f, r)
            # 후방 시야각 (150도 ~ -150도) 최소 거리
            elif deg >= 150 or deg <= -150:
                min_b = min(min_b, r)

        self.min_dist_front = min_f
        self.min_dist_back = min_b

    # --- 메인 주행 로직 ---
    def control_loop(self):
        current_time = time.time()
        cmd_msg = Twist()

        # Watchdog: 0.5초 안에 메시지가 새로 왔는지 확인
        target_valid = (current_time - self.last_target_time) <= self.watchdog_timeout
        search_valid = (current_time - self.last_search_time) <= self.watchdog_timeout

        state_msg = ""

        # [모드 1] 사람 추적 중 (/target_cmd 수신)
        if target_valid:
            error_dist = self.target_dist - self.target_distance
            linear_v = error_dist * 0.4  # P 제어 (거리)
            angular_v = self.target_angle * 1.0  # P 제어 (각도)

            cmd_msg.linear.x = max(min(linear_v, self.max_linear_vel), -self.max_linear_vel)
            cmd_msg.angular.z = max(min(angular_v, self.max_angular_vel), -self.max_angular_vel)
            state_msg = f"TRACKING (Dist: {self.target_dist:.2f}m)"

        # [모드 2] 사람 소실 및 탐색 중 (/search_cmd 수신)
        elif search_valid:
            cmd_msg.linear.x = 0.0  # 탐색 중에는 무조건 앞뒤 이동 정지
            if self.search_dir == "left":
                cmd_msg.angular.z = self.search_angular_vel
            elif self.search_dir == "right":
                cmd_msg.angular.z = -self.search_angular_vel
            state_msg = f"SEARCHING ({self.search_dir})"

        # [모드 3] 신호 끊김 (Watchdog 작동)
        else:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            state_msg = "IDLE / WATCHDOG STOP"

        # [최우선 순위] LiDAR 충돌 방지 오버라이드
        # 앞으로 가려는데 앞에 벽이 35cm 이내일 때
        if cmd_msg.linear.x > 0 and self.min_dist_front < self.safe_distance:
            cmd_msg.linear.x = 0.0
            state_msg += " [BLOCKED: FRONT]"
        # 뒤로 가려는데(사람이 다가와서) 뒤에 벽이 35cm 이내일 때
        elif cmd_msg.linear.x < 0 and self.min_dist_back < self.safe_distance:
            cmd_msg.linear.x = 0.0
            state_msg += " [BLOCKED: BACK]"

        # 모터 명령 최종 발행
        self.cmd_pub.publish(cmd_msg)

        # 현재 상태 화면 출력 (모니터링용)
        self.get_logger().info(
            f"{state_msg} -> V: {cmd_msg.linear.x:.2f}, W: {cmd_msg.angular.z:.2f} | "
            f"Lidar F: {self.min_dist_front:.2f}m, B: {self.min_dist_back:.2f}m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
