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
        
        self.target_sub = self.create_subscription(Twist, '/target_cmd', self.target_callback, 10)
        self.search_sub = self.create_subscription(String, '/search_cmd', self.search_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- 제어 파라미터 ---
        self.target_distance = 1.0  
        self.safe_distance = 0.45    
        
        self.kp_linear = 0.5        
        self.max_linear_speed = 0.4 
        self.deadband_linear = 0.15  
        
        self.kp_angular = 1.2       
        self.max_angular_speed = 0.8 
        self.deadband_angular = 0.05 
        
        self.search_speed = 0.12    
        
        # --- 상태 저장 변수 ---
        self.current_state = "TIMEOUT"  
        self.search_direction = ""
        self.last_state_str = "TIMEOUT" 
        
        self.needs_alignment = False    # [신규] 사람을 찾은 직후 조준(Align) 모드 진입 플래그
        
        self.current_distance = 0.0     
        self.target_angle_rad = 0.0
        
        self.min_dist_all = float('inf') # [신규] 360도 전방위 최소 거리
        self.last_msg_time = 0.0        

        self.log_counter = 0
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('\n' + '='*50 + '\n🚀 Person Follower (Turn-First Align & 360 LiDAR) Started!\n👉 Press "q" or "Ctrl+C" to cleanly stop.\n' + '='*50)

    def target_callback(self, msg):
        dist = msg.linear.x
        
        # 카메라 노이즈 필터링 (순간이동 방지)
        if dist <= 0.01 or dist > 3.0:
            return
        if self.current_distance > 0.1 and abs(dist - self.current_distance) > 0.8:
            return
            
        if self.current_distance < 0.1:
            self.current_distance = dist
        else:
            self.current_distance = (0.7 * self.current_distance) + (0.3 * dist)
            
        self.target_angle_rad = msg.angular.z
        
        self.last_msg_time = time.time()
        if self.current_state in ["TIMEOUT", "WAITING"]:
            self.current_state = "TRACKING"

    def search_callback(self, msg):
        command = msg.data
        self.last_msg_time = time.time()  
        
        if command in ["left", "right"]:
            self.current_state = "SEARCHING"
            self.search_direction = command
            
        elif command == "find":
            if self.current_state == "SEARCHING":
                brake_msg = Twist()
                self.publisher.publish(brake_msg)
                
                self.current_distance = self.target_distance  
                self.target_angle_rad = 0.0                   
                self.needs_alignment = True  # [핵심] 찾자마자 에임 정렬 모드 ON
                
            self.current_state = "TRACKING"
            self.search_direction = ""

    def scan_callback(self, msg):
        # [신규] 특정 각도를 제한하지 않고 360도 전체에서 가장 가까운 거리를 찾음
        min_d = float('inf')
        for r in msg.ranges:
            if 0.05 < r < 10.0 and not math.isinf(r) and not math.isnan(r):
                min_d = min(min_d, r)
        self.min_dist_all = min_d

    def log_state_change(self, new_state_str):
        if self.last_state_str != new_state_str:
            self.get_logger().info(f"\n======================================\n🔄 STATE CHANGED: {self.last_state_str} ➡️  {new_state_str}\n======================================")
            self.last_state_str = new_state_str

    def control_loop(self):
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

        if time.time() - self.last_msg_time > 0.5:
            self.current_state = "TIMEOUT"
            self.needs_alignment = False  # 신호 끊기면 정렬 모드도 해제

        if self.current_state == "TIMEOUT":
            current_state_str = "TIMEOUT (No Data)"
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            status_text = "NO_DATA"

        elif self.current_state == "SEARCHING":
            current_state_str = f"SEARCHING ({self.search_direction})"
            cmd_msg.linear.x = 0.0
            if self.search_direction == "left":
                cmd_msg.angular.z = self.search_speed
            elif self.search_direction == "right":
                cmd_msg.angular.z = -self.search_speed
            status_text = f"SEARCH"

        elif self.current_state == "TRACKING":
            # 1. 각도 계산 (공통)
            if abs(self.target_angle_rad) < self.deadband_angular:
                cmd_msg.angular.z = 0.0
            else:
                raw_angular_vel = self.target_angle_rad * self.kp_angular
                cmd_msg.angular.z = max(min(raw_angular_vel, self.max_angular_speed), -self.max_angular_speed)

            # 2. [신규] 정렬(Aligning) 모드: 직진을 막고 제자리 회전으로 중앙부터 맞춤
            if self.needs_alignment:
                current_state_str = "ALIGNING (Turn First)"
                status_text = "ALIGNING"
                cmd_msg.linear.x = 0.0  # 직진 완벽 차단
                
                # 정렬 중일 때는 오버슛 방지를 위해 회전 속도를 더 부드럽게 제한 (최대 0.3)
                cmd_msg.angular.z = max(min(cmd_msg.angular.z, 0.3), -0.3)
                
                # 오차가 0.15rad (약 8도) 이내로 들어오면 정렬 완료 판정
                if abs(self.target_angle_rad) <= 0.15:
                    self.needs_alignment = False
                    
            # 3. 일반 추종 모드: 정상 직진 허용
            else:
                distance_error = self.current_distance - self.target_distance
                if abs(distance_error) < self.deadband_linear:
                    cmd_msg.linear.x = 0.0
                else:
                    raw_linear_vel = distance_error * self.kp_linear
                    cmd_msg.linear.x = max(min(raw_linear_vel, self.max_linear_speed), -self.max_linear_speed)
                status_text = "TRACKING"

        # [신규] 360도 라이다 충돌 방지: 어느 방향이든 장애물이 너무 가까우면 이동(전/후진)만 차단
        if cmd_msg.linear.x != 0.0 and self.min_dist_all < self.safe_distance:
            cmd_msg.linear.x = 0.0
            status_text = "LIDAR_BLOCK"

        self.publisher.publish(cmd_msg)
        
        self.log_state_change(current_state_str)
        
        self.log_counter += 1
        if self.log_counter % 10 == 0:  
            self.get_logger().info(
                f'[{status_text:^11}] Dist: {self.current_distance:.2f}m | Vel: {cmd_msg.linear.x:+.2f} | '
                f'Ang: {self.target_angle_rad:+.2f}rad | AngVel: {cmd_msg.angular.z:+.2f} | '
                f'LiDAR 360\xb0: {self.min_dist_all:.2f}m'
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
