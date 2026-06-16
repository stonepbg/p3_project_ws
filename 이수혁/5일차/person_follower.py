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
        
        # --- 제어 파라미터 (튜닝값) ---
        self.target_distance = 1.0  
        self.safe_distance = 0.35   # 벽 충돌 방지 안전 거리
        
        # 선속도 제어 파라미터 (Linear)
        self.kp_linear = 0.5        
        self.max_linear_speed = 0.4 
        self.deadband_linear = 0.1  
        
        # 각속도 제어 파라미터 (Angular)
        self.kp_angular = 1.2       
        self.max_angular_speed = 0.8 
        self.deadband_angular = 0.05 
        
        # 탐색(Search) 파라미터
        self.search_speed = 0.12    # [수정] 오버슛 방지를 위해 기존 0.2에서 0.12로 대폭 감속
        
        # --- 상태 저장 변수 ---
        self.current_state = "TRACKING"  
        self.search_direction = ""
        
        self.current_distance = 1.0
        self.target_angle_rad = 0.0
        
        self.min_dist_front = float('inf')
        self.min_dist_back = float('inf')

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Person Follower Node (Hard Brake Applied) Started!')
        self.get_logger().info('*** Press "q" or "Ctrl+C" to cleanly stop the robot. ***')

    def target_callback(self, msg):
        self.current_distance = msg.linear.x
        self.target_angle_rad = msg.angular.z

    def search_callback(self, msg):
        command = msg.data
        if command in ["left", "right"]:
            self.current_state = "SEARCHING"
            self.search_direction = command
            
        elif command == "find":
            # [수정 핵심] FIND 신호를 받으면 즉시 하드 브레이크 (PWM 0 송출)
            if self.current_state == "SEARCHING":
                self.get_logger().info('🎯 TARGET FOUND! Applying Hard Brake.')
                
                # 1) 즉시 모터에 정지 명령 발행
                brake_msg = Twist()
                brake_msg.linear.x = 0.0
                brake_msg.angular.z = 0.0
                self.publisher.publish(brake_msg)
                
                # 2) 과거의 타겟 데이터 찌꺼기 초기화 (새 데이터가 올 때까지 P제어로 인한 회전 방지)
                self.current_distance = self.target_distance  # 오차를 0으로 만듦
                self.target_angle_rad = 0.0                   # 회전 각도를 0으로 만듦
                
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

            if -30 <= deg <= 30:
                min_f = min(min_f, r)
            elif deg >= 150 or deg <= -150:
                min_b = min(min_b, r)

        self.min_dist_front = min_f
        self.min_dist_back = min_b

    def control_loop(self):
        # 1. 키보드 'q' 입력 감지
        try:
            if select.select([sys.stdin], [], [], 0.0)[0]:
                char = sys.stdin.read(1)
                if char.lower() == 'q':
                    self.get_logger().warn("'q' key pressed! Initiating emergency stop...")
                    raise KeyboardInterrupt
        except ValueError:
            pass

        # 2. 메인 주행 로직
        cmd_msg = Twist()
        status_text = ""

        # [모드 A] 탐색 모드 (SEARCHING)
        if self.current_state == "SEARCHING":
            cmd_msg.linear.x = 0.0
            if self.search_direction == "left":
                cmd_msg.angular.z = self.search_speed
            elif self.search_direction == "right":
                cmd_msg.angular.z = -self.search_speed
            else:
                cmd_msg.angular.z = 0.0
            status_text = f"SEARCHING ({self.search_direction})"

        # [모드 B] 추종 모드 (TRACKING)
        elif self.current_state == "TRACKING":
            if self.current_distance <= 0.01:
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                status_text = "WAITING"
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

        # 3. 라이다 충돌 방지
        if cmd_msg.linear.x > 0 and self.min_dist_front < self.safe_distance:
            cmd_msg.linear.x = 0.0
            status_text += "[FRONT_BLOCKED]"
        elif cmd_msg.linear.x < 0 and self.min_dist_back < self.safe_distance:
            cmd_msg.linear.x = 0.0
            status_text += "[BACK_BLOCKED]"

        self.publisher.publish(cmd_msg)
        
        self.get_logger().info(
            f'[{status_text}] Dist: {self.current_distance:.2f}m -> Vel: {cmd_msg.linear.x:.2f} | '
            f'Ang: {self.target_angle_rad:.2f}rad -> AngVel: {cmd_msg.angular.z:.2f} | '
            f'Lidar F: {self.min_dist_front:.2f}m B: {self.min_dist_back:.2f}m'
        )

    def emergency_stop(self):
        self.get_logger().warn('EMERGENCY STOP: Halting all motors immediately...')
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
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
