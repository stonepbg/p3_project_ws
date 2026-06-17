#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String  
from rclpy.qos import qos_profile_sensor_data
import math
import time

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        
        self.subscription = self.create_subscription(Twist, '/target_cmd', self.target_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        # 제어 명령 퍼블리셔
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # [수정] 대시보드 로그 전송용 퍼블리셔 이름 변경
        self.status_pub = self.create_publisher(String, '/AGV_follower_status', 10)
        
        # --- 제어 및 안전 파라미터 ---
        self.target_distance = 1.0   
        self.safe_distance = 0.45    
        
        self.kp_linear = 0.4         
        self.max_linear_speed = 0.25 
        self.deadband_linear = 0.1
        
        self.kp_angular = 0.7        
        self.max_angular_speed = 0.5 
        self.deadband_angular = 0.08 
        
        self.search_speed = 0.3       
        self.step_angle_deg = 10.0    
        self.step_angle_rad = math.radians(self.step_angle_deg)
        self.search_rotate_duration = self.step_angle_rad / self.search_speed
        
        # 라이다 데이터
        self.min_dist_front = float('inf')
        self.min_dist_left = float('inf')
        self.min_dist_right = float('inf')
        
        self.last_msg_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.current_mode_text = "🟢 INIT"
        
        self.is_searching = False
        self.search_start_time = 0.0
        self.search_direction = 0.0 
        
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        self.get_logger().info('Person Follower Node (Dashboard Publisher Added) started!')

    def scan_callback(self, msg):
        min_f = float('inf')
        min_l = float('inf')
        min_r = float('inf')

        for i, r in enumerate(msg.ranges):
            if r < 0.05 or r > 10.0 or math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            deg = math.degrees(angle)

            if -40 <= deg <= 40:
                min_f = min(min_f, r)
            elif 40 < deg <= 90:
                min_l = min(min_l, r)
            elif -90 <= deg < -40:
                min_r = min(min_r, r)

        self.min_dist_front = min_f
        self.min_dist_left = min_l
        self.min_dist_right = min_r

    def target_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        
        mode = msg.linear.z
        current_distance = msg.linear.x
        target_angle_rad = msg.angular.z
        
        cmd_msg = Twist()
        status_text = "🎯 TRACKING"

        # Mode 0.0 : 기존 추종 모드 + 능동 회피
        if mode == 0.0:
            self.is_searching = False 
            
            if self.min_dist_front < self.safe_distance:
                status_text = "🚨 AVOIDANCE"
                cmd_msg.linear.x = -0.15 
                
                if self.min_dist_left > self.min_dist_right:
                    cmd_msg.angular.z = 0.4  
                else:
                    cmd_msg.angular.z = -0.4 
            else:
                distance_error = current_distance - self.target_distance
                if abs(distance_error) < self.deadband_linear:
                    cmd_msg.linear.x = 0.0
                else:
                    raw_linear_vel = distance_error * self.kp_linear
                    cmd_msg.linear.x = max(min(raw_linear_vel, self.max_linear_speed), -self.max_linear_speed)

                if abs(target_angle_rad) < self.deadband_angular:
                    cmd_msg.angular.z = 0.0
                else:
                    raw_angular_vel = target_angle_rad * self.kp_angular
                    cmd_msg.angular.z = max(min(raw_angular_vel, self.max_angular_speed), -self.max_angular_speed)

        # Mode 1.0 / 2.0 : 탐색 모드
        elif mode == 1.0 or mode == 2.0:
            status_text = "🔍⬅️ SEARCH_L" if mode == 1.0 else "🔍➡️ SEARCH_R"
            direction = 1.0 if mode == 1.0 else -1.0
            current_time = time.time()
            total_step_duration = self.search_rotate_duration + 0.5 
            
            if not self.is_searching or (current_time - self.search_start_time > total_step_duration):
                self.is_searching = True
                self.search_start_time = current_time
                self.search_direction = direction

            elapsed_time = current_time - self.search_start_time
            
            if elapsed_time < self.search_rotate_duration:
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = self.search_direction * self.search_speed
                status_text += "_ROT"
            else:
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                status_text += "_WAIT"

        # Mode 3.0 : 발견 및 정지
        elif mode == 3.0:
            self.is_searching = False
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            status_text = "🛑 FIND_STOP"

        self.publisher.publish(cmd_msg)
        self.current_mode_text = status_text
        self.print_clean_log(mode, current_distance, target_angle_rad, cmd_msg)

    def watchdog_check(self):
        now = self.get_clock().now()
        time_diff = (now - self.last_msg_time).nanoseconds / 1e9
        
        if time_diff > 0.5:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.publisher.publish(stop_msg)
            
            log_diff = (now - self.last_log_time).nanoseconds / 1e9
            if log_diff > 0.5:
                warn_str = f'[💀 WATCHDOG] No signal! EMERGENCY STOP. ({time_diff:.2f}s)'
                self.get_logger().warn(warn_str)
                
                status_msg = String()
                status_msg.data = warn_str
                self.status_pub.publish(status_msg)
                
                self.last_log_time = now

    def print_clean_log(self, mode, dist, angle, cmd):
        now = self.get_clock().now()
        log_diff = (now - self.last_log_time).nanoseconds / 1e9
        
        if log_diff >= 0.5:
            log_str = (
                f"[{self.current_mode_text:<15}] "
                f"Dist: {dist:.2f}m, Ang: {angle:.2f}rad | "
                f"Vel: {cmd.linear.x:+.2f}, Omeg: {cmd.angular.z:+.2f} | "
                f"Lidar(F/L/R): {self.min_dist_front:.1f}/{self.min_dist_left:.1f}/{self.min_dist_right:.1f}m"
            )
            self.get_logger().info(log_str)
            
            status_msg = String()
            status_msg.data = log_str
            self.status_pub.publish(status_msg)
            
            self.last_log_time = now

    def emergency_stop(self):
        warn_str = '💀 EMERGENCY STOP: Halting all motors...'
        self.get_logger().warn(warn_str)
        
        status_msg = String()
        status_msg.data = warn_str
        self.status_pub.publish(status_msg)
        
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
