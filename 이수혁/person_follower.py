import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data  # [수정] 라이다 수신용 특별 통신 규격
import math
import time

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        # 1. Publisher & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_sub = self.create_subscription(Twist, '/target_cmd', self.target_callback, 10)
        self.search_sub = self.create_subscription(String, '/search_cmd', self.search_callback, 10)
        
        # [수정] QoS 프로파일 적용 (이게 있어야 라이다 데이터를 정상적으로 받습니다!)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # 2. 로봇 제어 파라미터
        self.target_distance = 1.0      
        self.safe_distance = 0.35       
        self.max_linear_vel = 0.4       
        self.max_angular_vel = 0.5      
        self.search_angular_vel = 0.4   
        self.watchdog_timeout = 0.5     
        
        # [추가] 데드존 (허용 오차 범위)
        self.dist_deadzone = 0.15       # 목표 거리 +- 15cm 이내면 전진/후진 정지
        self.angle_deadzone = 0.1       # 각도 오차 0.1rad 이내면 회전 정지

        # 3. 상태 저장 변수들
        self.last_target_time = 0.0
        self.last_search_time = 0.0
        self.target_dist = 0.0
        self.target_angle = 0.0
        self.search_dir = ""
        self.min_dist_front = float('inf')
        self.min_dist_back = float('inf')

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Safe Person Follower Node Started!')

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
            # 노이즈 필터링
            if r < 0.05 or r > 10.0 or math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            deg = math.degrees(angle)

            if -30 <= deg <= 30:
                min_f = min(min_f, r)
            elif deg >= 150 or deg <= -150:
                min_b = min(min_b, r)

        self.min_dist_front = min_f
        self.min_dist_back = min_b

    # --- 긴급 정지 함수 (Ctrl+C를 누를 때 발동) ---
    def emergency_stop(self):
        self.get_logger().info('EMERGENCY STOP: Halting all motors...')
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)
        time.sleep(0.1) # 모터가 명령을 받을 수 있도록 0.1초 대기

    # --- 메인 주행 로직 ---
    def control_loop(self):
        current_time = time.time()
        cmd_msg = Twist()

        target_valid = (current_time - self.last_target_time) <= self.watchdog_timeout
        search_valid = (current_time - self.last_search_time) <= self.watchdog_timeout

        state_msg = ""

        # [모드 1] 사람 추적 중
        if target_valid:
            error_dist = self.target_dist - self.target_distance
            
            # [추가] 거리 데드존 적용 (15cm 이내면 정지)
            if abs(error_dist) < self.dist_deadzone:
                linear_v = 0.0
            else:
                linear_v = error_dist * 0.4  

            # [추가] 각도 데드존 적용
            if abs(self.target_angle) < self.angle_deadzone:
                angular_v = 0.0
            else:
                angular_v = self.target_angle * 1.0  

            cmd_msg.linear.x = max(min(linear_v, self.max_linear_vel), -self.max_linear_vel)
            cmd_msg.angular.z = max(min(angular_v, self.max_angular_vel), -self.max_angular_vel)
            state_msg = f"TRACKING (Dist: {self.target_dist:.2f}m)"

        # [모드 2] 사람 소실 및 탐색 중
        elif search_valid:
            cmd_msg.linear.x = 0.0  
            if self.search_dir == "left":
                cmd_msg.angular.z = self.search_angular_vel
            elif self.search_dir == "right":
                cmd_msg.angular.z = -self.search_angular_vel
            state_msg = f"SEARCHING ({self.search_dir})"

        # [모드 3] 신호 끊김
        else:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            state_msg = "IDLE / WATCHDOG STOP"

        # [최우선 순위] LiDAR 충돌 방지
        if cmd_msg.linear.x > 0 and self.min_dist_front < self.safe_distance:
            cmd_msg.linear.x = 0.0
            state_msg += " [🚨 BLOCKED: FRONT]"
        elif cmd_msg.linear.x < 0 and self.min_dist_back < self.safe_distance:
            cmd_msg.linear.x = 0.0
            state_msg += " [🚨 BLOCKED: BACK]"

        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info(f"{state_msg} | F:{self.min_dist_front:.2f}m B:{self.min_dist_back:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C가 눌렸을 때 예외 처리로 들어옴
        node.get_logger().warn('Ctrl+C detected! Shutting down safely...')
    finally:
        # 노드가 파괴되기 전에 무조건 속도를 0으로 쏘고 죽음
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
