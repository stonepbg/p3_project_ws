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
        self.safe_distance = 0.35    # 라이다 충돌 방지 안전 거리
        
        # 직진 제어
        self.kp_linear = 0.5        
        self.max_linear_speed = 0.4 
        self.deadband_linear = 0.15  
        
        # 회전 제어 (사용자 튜닝값 0.9 복구)
        self.kp_angular = 0.9        
        self.max_angular_speed = 0.6 
        
        # [핵심] 직진/회전 모드 스위칭 기준 (0.15 rad = 약 8.5도)
        self.turn_threshold = 0.15  
        
        self.search_speed = 0.12    
        
        # --- 상태 저장 변수 ---
        self.current_state = "TIMEOUT"  
        self.search_direction = ""
        self.last_state_str = "TIMEOUT" 
        
        self.current_distance = 0.0     
        self.target_angle_rad = 0.0
        
        # 라이다 전/후 분리 센서 (회전은 절대 막지 않음)
        self.min_dist_front = float('inf')
        self.min_dist_back = float('inf')
        
        self.last_msg_time = time.time()        

        self.log_counter = 0
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('\n' + '='*50 + '\n🚀 Person Follower (Strict Turn-Then-Move & Zero Lag) Started!\n👉 Press "q" or "Ctrl+C" to cleanly stop.\n' + '='*50)

    def target_callback(self, msg):
        dist = msg.linear.x
        
        # 카메라 오작동 노이즈만 칼같이 컷트 (3.5m 초과 데이터 버림)
        if dist <= 0.01 or dist > 3.5:
            return
            
        # [핵심] 필터 삭제. 카메라 데이터를 0.1초의 지연도 없이 즉시 뇌로 전달
        self.current_distance = dist
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
                # 찾자마자 하드 브레이크로 헛도는 관성만 죽임
                brake_msg = Twist()
                self.publisher.publish(brake_msg)
            self.current_state = "TRACKING"
            self.search_direction = ""

    def scan_callback(self, msg):
        min_f = float('inf')
        min_b = float('inf')
        
        # 360도를 앞쪽 120도(-60~60)와 뒤쪽 120도(120~180, -180~-120)로 쪼개서 검사
        for i, r in enumerate(msg.ranges):
            if 0.05 < r < 10.0 and not math.isinf(r) and not math.isnan(r):
                angle = msg.angle_min + i * msg.angle_increment
                deg = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
                
                if -60 <= deg <= 60:
                    min_f = min(min_f, r)
                elif deg >= 120 or deg <= -120:
                    min_b = min(min_b, r)
                    
        self.min_dist_front = min_f
        self.min_dist_back = min_b

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

        if time.time() - self.last_msg_time > 1.0:
            self.current_state = "TIMEOUT"

        if self.current_state == "TIMEOUT":
            current_state_str = "TIMEOUT (No Data)"
            status_text = "NO_DATA"

        elif self.current_state == "SEARCHING":
            current_state_str = f"SEARCHING ({self.search_direction})"
            if self.search_direction == "left":
                cmd_msg.angular.z = self.search_speed
            elif self.search_direction == "right":
                cmd_msg.angular.z = -self.search_speed
            status_text = f"SEARCH"

        elif self.current_state == "TRACKING":
            # [핵심] 직진과 회전을 완벽하게 분리 (Turn-then-Move)
            
            # 1. 회전 모드: 오차가 8.5도(0.15 rad) 이상이면 직진 포기하고 제자리 회전만 수행
            if abs(self.target_angle_rad) > self.turn_threshold:
                cmd_msg.linear.x = 0.0
                raw_angular_vel = self.target_angle_rad * self.kp_angular
                cmd_msg.angular.z = max(min(raw_angular_vel, self.max_angular_speed), -self.max_angular_speed)
                status_text = "TRACK_TURN"
                current_state_str = "TRACKING (Turning)"
                
            # 2. 직진 모드: 사람을 정면에 맞췄으면 회전을 멈추고 100% 직선으로만 돌격
            else:
                cmd_msg.angular.z = 0.0  # 회전 원천 차단 (직진 중 흔들림 제거)
                distance_error = self.current_distance - self.target_distance
                
                if abs(distance_error) < self.deadband_linear:
                    cmd_msg.linear.x = 0.0
                else:
                    raw_linear_vel = distance_error * self.kp_linear
                    cmd_msg.linear.x = max(min(raw_linear_vel, self.max_linear_speed), -self.max_linear_speed)
                status_text = "TRACK_MOVE"
                current_state_str = "TRACKING (Moving Straight)"

        # [핵심] 라이다 방향별 안전장치 
        # (앞이 막히면 직진만, 뒤가 막히면 후진만 금지. 제자리 회전은 영원히 허용되어 스스로 탈출 가능)
        if cmd_msg.linear.x > 0 and self.min_dist_front < self.safe_distance:
            cmd_msg.linear.x = 0.0
            status_text = "LIDAR_F_BLK"
            current_state_str = "LIDAR_BLOCKED (Front)"
            
        elif cmd_msg.linear.x < 0 and self.min_dist_back < self.safe_distance:
            cmd_msg.linear.x = 0.0
            status_text = "LIDAR_B_BLK"
            current_state_str = "LIDAR_BLOCKED (Back)"

        self.publisher.publish(cmd_msg)
        self.log_state_change(current_state_str)
        
        self.log_counter += 1
        if self.log_counter % 10 == 0:  
            self.get_logger().info(
                f'[{status_text:^11}] Dist: {self.current_distance:.2f}m | Vel: {cmd_msg.linear.x:+.2f} | '
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
