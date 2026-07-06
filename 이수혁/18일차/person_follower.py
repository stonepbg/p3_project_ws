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
        self.kp_angular = 0.4        
        self.max_angular_speed = 0.5 
        self.deadband_angular = 0.12 
        
        self.search_speed = 0.3        
        self.search_rotate_duration = math.radians(10.0) / self.search_speed
        
        self.min_dist_front = float('inf')
        self.min_dist_left = float('inf')
        self.min_dist_right = float('inf')
        
        self.last_known_angle = 0.0 
        
        self.last_msg_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.mode_start_time = self.get_clock().now()
        
        self.current_mode_text = "🟢 INIT"
        
        self.is_searching = False
        self.search_start_time = 0.0
        self.search_direction = 0.0 
        
        self.is_auto_searching = False
        self.auto_search_direction = 1.0
        
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        self.get_logger().info('Person Follower Node (FSM 연동 최적화 및 0.5초 정밀 끊어치기 탐색) started!')

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
            self.is_auto_searching = False
            self.last_msg_time = self.get_clock().now()
            self.mode_start_time = self.get_clock().now()
            self.send_log('▶️ 10번 모드 진입: 1.0m 간격 사람 추종을 시작합니다.')
            
        elif self.current_agv_mode == 50 and prev_mode != 50:
            self.target_distance = 0.30  
            self.safe_distance = 0.20    
            self.is_auto_searching = False
            self.last_msg_time = self.get_clock().now()
            self.mode_start_time = self.get_clock().now()
            self.send_log('▶️ 50번 모드 진입: 7초간 타겟 대기 후 픽업 유도를 시작합니다.')
            
        elif prev_mode in [10, 50] and self.current_agv_mode not in [10, 50]:
            self.send_log('⏹️ 추종/유도 모드 해제: 즉시 브레이크를 작동합니다.')
            self.is_auto_searching = False
            self.emergency_stop()

    def scan_callback(self, msg):
        if self.current_agv_mode not in [10, 50]: return
        
        min_f, min_l, min_r = float('inf'), float('inf'), float('inf')

        for i, r in enumerate(msg.ranges):
            if r < 0.05 or r > 10.0 or math.isinf(r) or math.isnan(r): continue
            
            angle = msg.angle_min + i * msg.angle_increment
            raw_deg = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
            real_front_deg = raw_deg + 180.0
            if real_front_deg > 180.0:
                real_front_deg -= 360.0

            if self.current_agv_mode == 10:
                if -40 <= real_front_deg <= 40: min_f = min(min_f, r)
                elif 40 < real_front_deg <= 90: min_l = min(min_l, r)
                elif -90 <= real_front_deg < -40: min_r = min(min_r, r)
            elif self.current_agv_mode == 50:
                if -20 <= real_front_deg <= 20: min_f = min(min_f, r)
                elif 20 < real_front_deg <= 60: min_l = min(min_l, r)
                elif -60 <= real_front_deg < -20: min_r = min(min_r, r)

        self.min_dist_front, self.min_dist_left, self.min_dist_right = min_f, min_l, min_r

    def target_callback(self, msg):
        if self.current_agv_mode not in [10, 50]: return
        
        if self.is_auto_searching:
            self.send_log('✅ 대상을 다시 발견했습니다! 유도를 재개합니다.')
            self.is_auto_searching = False
            
        self.last_msg_time = self.get_clock().now()
        
        mode = msg.linear.z
        current_distance = msg.linear.x
        target_angle_rad = msg.angular.z
        
        self.last_known_angle = target_angle_rad 
        
        cmd_msg = Twist()
        status_text = f"🎯 TRACKING ({int(self.target_distance*100)}cm)"

        if mode == 0.0:
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
                cmd_msg.angular.z = 0.0 
                status_text += "_WAIT"

        self.publisher.publish(cmd_msg)
        self.current_mode_text = status_text
        self.print_clean_log(current_distance, target_angle_rad)

    def watchdog_check(self):
        if self.current_agv_mode not in [10, 50]: return
        
        now_time = self.get_clock().now()
        time_diff = (now_time - self.last_msg_time).nanoseconds / 1e9
        time_since_start = (now_time - self.mode_start_time).nanoseconds / 1e9
        
        # 50번 픽업 모드 진입 시 초기 7초간 대기 (이때는 통신이 끊겨도 회전하지 않음)
        if self.current_agv_mode == 50 and time_since_start < 7.0:
            if time_diff > 1.0: 
                cmd_msg = Twist()
                self.publisher.publish(cmd_msg)
                if (now_time - self.last_log_time).nanoseconds / 1e9 >= 0.5:
                    self.current_mode_text = f"⏳ INITIAL_WAIT ({7.0 - time_since_start:.1f}s)"
                    log_str = f"[{self.current_mode_text}] 신호 대기 중... | Lidar: {self.min_dist_front:.1f}m"
                    self.send_log(log_str)
                    self.last_log_time = now_time
            return 
            
        # 초기 대기가 끝났거나 10번 모드일 때, 1초 이상 데이터가 안 들어오면 자동 탐색 시작
        if time_diff > 1.0:
            if not self.is_auto_searching:
                self.is_auto_searching = True
                self.auto_search_direction = 1.0 if self.last_known_angle >= 0 else -1.0
                self.search_start_time = time.time()
                
                dir_str = "좌측" if self.auto_search_direction > 0 else "우측"
                self.send_log(f'⚠️ 대상 유실! 대상을 다시 찾을 때까지 마지막 위치({dir_str}) 방향으로 무한 탐색을 시작합니다.', 'warn')

            if self.is_auto_searching:
                cmd_msg = Twist()
                current_sys_time = time.time()
                total_step_duration = self.search_rotate_duration + 0.5

                if current_sys_time - self.search_start_time > total_step_duration:
                    self.search_start_time = current_sys_time

                # 10도 회전(0~0.58초) 후 정지(0.58~1.08초) 반복
                if current_sys_time - self.search_start_time < self.search_rotate_duration:
                    cmd_msg.angular.z = self.auto_search_direction * self.search_speed
                else:
                    cmd_msg.angular.z = 0.0 

                self.publisher.publish(cmd_msg)

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
