#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math
import time
import sys
import select
import termios
import tty

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        
        # 1. Subscribers & Publisher
        self.target_sub = self.create_subscription(Twist, '/target_cmd', self.target_callback, 10)
        self.search_sub = self.create_subscription(String, '/search_cmd', self.search_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- 제어 파라미터 ---
        self.target_distance = 1.0  
        self.safe_distance = 0.45    # [수정] 충돌 방지 거리 증가 (0.35 -> 0.45)
        
        self.kp_linear = 0.5        
        self.max_linear_speed = 0.4 
        self.deadband_linear = 0.15  # [수정] 예민한 후진 방지를 위해 데드존 확대 (15cm)
        
        self.kp_angular = 1.2       
        self.max_angular_speed = 0.8 
        self.deadband_angular = 0.05 
        
        self.search_speed = 0.12    
        
        # --- 상태 저장 변수 ---
        self.current_state = "WAITING"  
        self.search_direction = ""
        self.last_state_str = "WAITING"  # 로그 상태 변화 감지용
        
        self.current_distance = 1.0
        self.target_angle_rad = 0.0
        self.min_dist_front = float('inf')
        self.min_dist_back = float('inf')

        self.log_counter = 0

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('\n' + '='*50 + '\n🚀 Person Follower Node (Anti-Noise & Wide LiDAR) Started!\n👉 Press "q" or "Ctrl+C" to cleanly stop the robot.\n' + '='*50)

    def target_callback(self, msg):
        dist = msg.linear.x
        
        # [핵심] AI 카메라 노이즈 필터링 (순간이동 방지)
        if dist > 3.0:
            # 3m 이상의 거리가 갑자기 들어오면 YOLO 오인식으로 간주하고 무시
            return
            
        self.current_distance = dist
        self.target_angle_rad = msg.angular.z

    def search_callback(self, msg):
        command = msg.data
        if command in ["left", "right"]:
            self.current_state = "SEARCHING"
            self.search_direction = command
            
        elif command == "find":
            if self.current_state == "SEARCHING":
                # 하드 브레이크 (관성 죽이기)
                brake_msg = Twist()
                self.publisher.publish(brake_msg)
                
                self.current_distance = self.target_distance  
                self.target_angle_rad = 0.0                   
                
            self.current_state = "TRACKING"
            self.search_direction = ""

    def scan_callback(self, msg):
        min_f = float('inf')
        min_b = float('inf')

        for i, r in enumerate(msg.ranges):
            if r < 0.05 or r > 10.0 or math.isinf(r) or math.isnan(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            deg = math.degrees(angle)

            # [수정] 라이다 시야각 확대 (정면/후면 모두 ±50도로 넓혀서 대각선 장애물 감지)
            if -50 <= deg <= 50:
                min_f = min(min_f, r)
            elif deg >= 130 or deg <= -130:
                min_b = min(min_b, r)

        self.min_dist_front = min_f
        self.min_dist_back = min_b

    def log_state_change(self, new_state_str):
        if self.last_state_str != new_state_str:
            self.get_logger().info(f"\n======================================\n🔄 STATE CHANGED: {self.last_state_str} ➡️  {new_state_str}\n======================================")
            self.last_state_str = new_state_str

    def control_loop(self):
        # 1. 키보드 'q' 감지
        try:
            if select.select([sys.stdin], [], [], 0.0)[0]:
                char = sys.stdin.read(1)
                if char.lower() == 'q':
                    self.get_logger().warn("\n🚨 'q' key pressed! Initiating emergency stop...")
                    raise KeyboardInterrupt
        except ValueError:
            pass

        cmd_msg = Twist()
        status_text = ""
        current_state_str = self.current_state

        # [모드 A] 탐색 모드 (SEARCHING)
        if self.current_state == "SEARCHING":
            current_state_str = f"SEARCHING ({self.search_direction})"
            cmd_msg.linear.x = 0.0
            if self.search_direction == "left":
                cmd_msg.angular.z = self.search_speed
            elif self.search_direction == "right":
                cmd_msg.angular.z = -self.search_speed
            status_text = f"SEARCH ({self.search_direction})"

        # [모드 B] 추종 모드 (TRACKING)
        elif self.current_state == "TRACKING":
            if self.current_distance <= 0.01:
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                status_text = "STANDBY"
                current_state_str = "STANDBY (Dist 0)"
            else:
                distance_error = self.current_distance - self.target_distance
                if abs(distance_error) < self.deadband_linear:
                    cmd_msg.linear.x = 0.0
                else:
                    raw_linear_vel = distance_error * self.kp_linear
                    cmd_msg.linear.x = max(min(raw_linear_vel, self.max_linear_speed), -self.max_linear_speed)

                if abs(self.target_angle_rad) < self.deadband_angular:
                    cmd_msg.angular.z = 0.0
                else:
                    raw_angular_vel = self.target_angle_rad * self.kp_angular
                    cmd_msg.angular.z = max(min(raw_angular_vel, self.max_angular_speed), -self.max_angular_speed)
                status_text = "TRACKING"
                
        else:
            status_text = "WAITING"

        # 3. 라이다 충돌 방지 오버라이드
        if cmd_msg.linear.x > 0 and self.min_dist_front < self.safe_distance:
            cmd_msg.linear.x = 0.0
            status_text = "FRONT_BLOCKED"
        elif cmd_msg.linear.x < 0 and self.min_dist_back < self.safe_distance:
            cmd_msg.linear.x = 0.0
            status_text = "BACK_BLOCKED"

        self.publisher.publish(cmd_msg)
        
        # 4. 스마트 로깅 (상태 변화 알림 & 0.5초 단위 요약 출력)
        self.log_state_change(current_state_str)
        
        self.log_counter += 1
        if self.log_counter % 10 == 0:  # 0.05 * 10 = 0.5초마다 출력
            self.get_logger().info(
                f'[{status_text:^13}] Dist: {self.current_distance:.2f}m | Vel: {cmd_msg.linear.x:+.2f} | '
                f'Ang: {self.target_angle_rad:+.2f}rad | AngVel: {cmd_msg.angular.z:+.2f} | '
                f'Lidar F: {self.min_dist_front:.2f}m B: {self.min_dist_back:.2f}m'
            )

    def emergency_stop(self):
        self.get_logger().warn('\n🛑 EMERGENCY STOP: Halting all motors immediately...')
        stop_msg = Twist()
        self.publisher.publish(stop_msg)
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()

    old_settings = None
    try:
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    except Exception:
        pass

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except Exception:
                pass
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
