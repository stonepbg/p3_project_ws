import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32
from rclpy.qos import qos_profile_sensor_data
import math
import time

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        
        self.mode_sub = self.create_subscription(Int32, '/internal_mode', self.mode_callback, 10)
        self.subscription = self.create_subscription(Twist, '/target_cmd', self.target_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.log_pub = self.create_publisher(String, '/AGV_log', 10)
        
        self.current_agv_mode = 0 
        self.target_distance = 1.0   
        self.safe_distance = 0.45    
        
        self.kp_linear = 0.4         
        self.max_linear_speed = 0.25 
        self.deadband_linear = 0.1
        self.kp_angular = 0.7        
        self.max_angular_speed = 0.5 
        self.deadband_angular = 0.08 
        
        self.search_speed = 0.3        
        self.search_rotate_duration = math.radians(10.0) / self.search_speed
        
        self.min_dist_front = float('inf')
        self.min_dist_left = float('inf')
        self.min_dist_right = float('inf')
        
        # [신규] 타겟을 놓쳤을 때 찾기 위해 마지막 각도를 기억하는 변수
        self.last_known_angle = 0.0 
        
        self.last_msg_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.current_mode_text = "🟢 INIT"
        
        self.is_searching = False
        self.search_start_time = 0.0
        self.search_direction = 0.0 
        
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        self.get_logger().info('Person Follower Node (스마트 탐색 및 모드별 라이다 분리 적용) started!')

    def send_log(self, text, level='info'):
        if level == 'info': self.get_logger().info(text)
        elif level == 'warn': self.get_logger().warn(text)
        elif level == 'error': self.get_logger().error(text)
        
        msg = String()
        msg.data = f"[추종/유도] {text}"
        self.log_pub.publish(msg)

    def mode_callback(self, msg):
        prev_mode = self.current_agv_mode
        self.current_agv_mode = msg.data

        if self.current_agv_mode == 10 and prev_mode != 10:
            self.target_distance = 1.0
            self.safe_distance = 0.45
            self.send_log('▶️ 10번 모드 진입: 1.0m 간격 사람 추종을 시작합니다.')
            now = self.get_clock().now()
            self.last_msg_time = now
            self.last_log_time = now
            
        elif self.current_agv_mode == 50 and prev_mode != 50:
            self.target_distance = 0.30  
            self.safe_distance = 0.20    
            self.send_log('▶️ 50번 모드 진입: 30cm 근접 픽업 유도를 시작합니다.')
            now = self.get_clock().now()
            self.last_msg_time = now
            self.last_log_time = now
            
        elif prev_mode in [10, 50] and self.current_agv_mode not in [10, 50]:
            self.send_log('⏹️ 추종/유도 모드 해제: 즉시 브레이크를 작동합니다.')
            self.emergency_stop()

    def scan_callback(self, msg):
        if self.current_agv_mode not in [10, 50]: return
        
        min_f, min_l, min_r = float('inf'), float('inf'), float('inf')

        for i, r in enumerate(msg.ranges):
            if r < 0.05 or r > 10.0 or math.isinf(r) or math.isnan(r): continue
            angle = msg.angle_min + i * msg.angle_increment
            deg = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))

            # [핵심] 10번 모드와 50번 모드의 라이다 시야각 분리
            if self.current_agv_mode == 10:
                # 일반 추종: 넓은 시야 (전방 ±40도)
                if -40 <= deg <= 40: min_f = min(min_f, r)
                elif 40 < deg <= 90: min_l = min(min_l, r)
                elif -90 <= deg < -40: min_r = min(min_r, r)
            elif self.current_agv_mode == 50:
                # 픽업 유도: 좁은 시야 (전방 ±20도) - 주변 사물 오인 방지
                if -20 <= deg <= 20: min_f = min(min_f, r)
                elif 20 < deg <= 60: min_l = min(min_l, r)
                elif -60 <= deg < -20: min_r = min(min_r, r)

        self.min_dist_front, self.min_dist_left, self.min_dist_right = min_f, min_l, min_r

    def target_callback(self, msg):
        if self.current_agv_mode not in [10, 50]: return
        self.last_msg_time = self.get_clock().now()
        
        mode = msg.linear.z
        current_distance = msg.linear.x
        target_angle_rad = msg.angular.z
        
        cmd_msg = Twist()
        status_text = f"🎯 TRACKING ({int(self.target_distance*100)}cm)"

        if mode == 0.0:
            # 타겟을 보고 있을 때 마지막 각도를 지속적으로 기억
            self.last_known_angle = target_angle_rad 
            self.is_searching = False 
            
            if self.min_dist_front < self.safe_distance:
                status_text = "🚨 AVOIDANCE"
                cmd_msg.linear.x = -0.15 
                if self.min_dist_left > self.min_dist_right: cmd_msg.angular.z = 0.4  
                else: cmd_msg.angular.z = -0.4 
            else:
                distance_error = current_distance - self.target_distance
                if abs(distance_error) < self.deadband_linear: cmd_msg.linear.x = 0.0
                else: cmd_msg.linear.x = max(min(distance_error * self.kp_linear, self.max_linear_speed), -self.max_linear_speed)

                if abs(target_angle_rad) < self.deadband_angular: cmd_msg.angular.z = 0.0
                else: cmd_msg.angular.z = max(min(target_angle_rad * self.kp_angular, self.max_angular_speed), -self.max_angular_speed)

        elif mode == 1.0 or mode == 2.0:
            status_text = "🔍⬅️ SEARCH_L" if mode == 1.0 else "🔍➡️ SEARCH_R"
            direction = 1.0 if mode == 1.0 else -1.0
            current_time = time.time()
            total_step_duration = self.search_rotate_duration + 0.5 
            
            if not self.is_searching or (current_time - self.search_start_time > total_step_duration):
                self.is_searching = True
                self.search_start_time = current_time
                self.search_direction = direction

            if current_time - self.search_start_time < self.search_rotate_duration:
                cmd_msg.angular.z = self.search_direction * self.search_speed
                status_text += "_ROT"
            else:
                status_text += "_WAIT"

        elif mode == 3.0:
            # FSM에서 3.0(놓침)을 명시적으로 보내올 경우 자율 탐색 시작
            self.is_searching = True
            direction = 1.0 if self.last_known_angle >= 0 else -1.0
            cmd_msg.angular.z = direction * self.search_speed
            status_text = "🔍⬅️ AUTO_SEARCH" if direction > 0 else "🔍➡️ AUTO_SEARCH"

        self.publisher.publish(cmd_msg)
        self.current_mode_text = status_text
        self.print_clean_log(current_distance, target_angle_rad)

    def watchdog_check(self):
        if self.current_agv_mode not in [10, 50]: return
        now = self.get_clock().now()
        time_diff = (now - self.last_msg_time).nanoseconds / 1e9
        
        # [신규] 신호가 끊겼을 때 (0.5초 ~ 5.0초 사이) -> 즉시 정지하지 않고 마지막 위치로 회전 탐색
        if 0.5 < time_diff < 5.0:
            cmd_msg = Twist()
            direction = 1.0 if self.last_known_angle >= 0 else -1.0
            cmd_msg.angular.z = direction * self.search_speed
            self.publisher.publish(cmd_msg)
            
            if (now - self.last_log_time).nanoseconds / 1e9 > 1.0:
                dir_str = "좌측" if direction > 0 else "우측"
                self.send_log(f'⚠️ 대상 유실! 마지막 위치({dir_str})로 회전하며 탐색합니다. ({time_diff:.1f}s)', 'warn')
                self.last_log_time = now
                
        # 5초 이상 회전해도 못 찾으면 포기하고 정지
        elif time_diff >= 5.0:
            self.emergency_stop() 
            if (now - self.last_log_time).nanoseconds / 1e9 > 2.0:
                self.send_log('💀 5초 이상 대상 유실. 탐색을 포기하고 대기합니다.', 'error')
                self.last_log_time = now

    def print_clean_log(self, dist, angle):
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 >= 0.5:
            log_str = f"[{self.current_mode_text}] Dist: {dist:.2f}m, Ang: {angle:.2f}rad | Lidar: {self.min_dist_front:.1f}m"
            self.send_log(log_str)
            self.last_log_time = now

    def emergency_stop(self):
        stop_msg = Twist()
        for _ in range(3):
            self.publisher.publish(stop_msg)
            time.sleep(0.05)

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()
