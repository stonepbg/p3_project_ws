import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from rclpy.qos import qos_profile_sensor_data
import math
import time

class Nav2Commander(Node):
    def __init__(self):
        super().__init__('nav2_commander')

        self.mode_sub = self.create_subscription(Int32, '/internal_mode', self.mode_callback, 10)
        # [수정 완상복구] 안내 모드 후방 사람 유무 확인용 토픽
        self.person_sub = self.create_subscription(Bool, '/rear_person', self.person_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        self.status_pub = self.create_publisher(Int32, '/AGV_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.log_pub = self.create_publisher(String, '/AGV_log', 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 8번 마커가 포함된 최신 데이터베이스
        self.marker_database = {
            0: (0.055, -0.382, 3.12), 1: (1.759, -0.202, 1.57),
            2: (1.457, -2.864, 0.03), 3: (1.567, -4.364, 0.01),
            4: (1.561, -5.816, 0.00), 5: (1.440, -7.275, 0.00),
            6: (1.485, -8.477, -1.57), 7: (0.142, -8.044, -3.13),
            8: (1.611, -2.843, 3.14)
        }

        self.current_mode = 0
        self.current_target_id = None
        self.goal_handle = None
        self.is_paused = False
        
        # 최신 라이다 회피 설정
        self.danger_zone_front = 0.25   
        self.danger_zone_corner = 0.18  
        self.warning_zone_front = 0.9   
        self.warning_zone_corner = 0.45  
        
        self.is_avoiding = False
        self.is_detouring = False 
        self.avoidance_start_time = 0.0
        self.avoidance_duration = 1.5
        
        self.detour_last_seen_time = 0.0 
        self.mission_completed = False
        
        self.status_timer = self.create_timer(0.5, self.publish_status_loop)

        self.get_logger().info('🗺️ Nav2 커맨더 가동 (/rear_person 토픽 기반 안내모드 제어 적용 완료)')

    def send_log(self, text, level='info'):
        if level == 'info': self.get_logger().info(text)
        elif level == 'warn': self.get_logger().warn(text)
        elif level == 'error': self.get_logger().error(text)
        
        msg = String()
        msg.data = f"[주행] {text}"
        self.log_pub.publish(msg)

    def publish_status_loop(self):
        if self.mission_completed:
            msg = Int32()
            msg.data = 1
            self.status_pub.publish(msg)

    def scan_callback(self, msg):
        if self.current_mode == 0 or self.current_mode == 10 or self.is_paused:
            return

        current_time = time.time()
        
        min_d_f, min_d_l, min_d_r = float('inf'), float('inf'), float('inf') 
        min_w_f, min_w_l, min_w_r = float('inf'), float('inf'), float('inf') 

        for i, r in enumerate(msg.ranges):
            if r < 0.05 or r > 10.0 or math.isinf(r) or math.isnan(r): continue
            angle = msg.angle_min + i * msg.angle_increment
            deg = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
            
            if -12 <= deg <= 12: min_d_f = min(min_d_f, r)
            elif 12 < deg <= 25: min_d_l = min(min_d_l, r)
            elif -25 <= deg < -12: min_d_r = min(min_d_r, r)
            
            if -15 <= deg <= 15: min_w_f = min(min_w_f, r)
            elif 15 < deg <= 40: min_w_l = min(min_w_l, r)
            elif -40 <= deg < -15: min_w_r = min(min_w_r, r)

        is_danger = (min_d_f < self.danger_zone_front or min_d_l < self.danger_zone_corner or min_d_r < self.danger_zone_corner)
        is_safe = (min_d_f > self.danger_zone_front + 0.15 and min_d_l > self.danger_zone_corner + 0.1 and min_d_r > self.danger_zone_corner + 0.1)
        
        is_warning = (min_w_f < self.warning_zone_front or min_w_l < self.warning_zone_corner or min_w_r < self.warning_zone_corner)

        if not self.is_avoiding:
            if is_danger:
                self.send_log(f'🚨 긴급 충돌 위험! ({self.avoidance_duration}초간 회피 기동)', 'warn')
                self.cancel_current_goal()
                self.is_avoiding = True
                self.is_detouring = False 
                self.avoidance_start_time = current_time
                
                for _ in range(2):
                    status_msg = Int32()
                    status_msg.data = 2
                    self.status_pub.publish(status_msg)
                
            elif is_warning:
                if not self.is_detouring:
                    self.send_log('⚠️ 전방 장애물 감지! 2번 신호를 1회 전송합니다.', 'warn')
                    self.is_detouring = True
                    
                    for _ in range(2):
                        status_msg = Int32()
                        status_msg.data = 2
                        self.status_pub.publish(status_msg)
                
                self.detour_last_seen_time = current_time
                
            elif not is_warning and self.is_detouring:
                if current_time - self.detour_last_seen_time > 3.0:
                    self.send_log('✅ 장애물을 완전히 통과했습니다. 정상 주행.')
                    self.is_detouring = False

        else:
            elapsed = current_time - self.avoidance_start_time
            if elapsed > self.avoidance_duration and is_safe:
                self.send_log('✅ 안전 공간 확보. 목적지로 재출발합니다.')
                self.cmd_vel_pub.publish(Twist()) 
                self.is_avoiding = False
                self.send_nav_goal(self.current_target_id)
            else:
                avoid_cmd = Twist()
                avoid_cmd.linear.x = -0.12
                if min_d_l < min_d_r: avoid_cmd.angular.z = -0.35
                else: avoid_cmd.angular.z = 0.35
                self.cmd_vel_pub.publish(avoid_cmd)

    def mode_callback(self, msg):
        mode = msg.data
        if mode == self.current_mode: return 
        self.current_mode = mode

        if mode == 0 or mode == 10:
            self.cancel_current_goal()
            return

        if 20 <= mode <= 27:
            self.current_target_id = mode - 20
            self.is_paused = False
            self.mission_completed = False
            self.send_log(f'💁 [안내 모드] {self.current_target_id}번 목적지로 안내를 시작합니다.')
            self.send_nav_goal(self.current_target_id)

        elif 30 <= mode <= 37:
            self.current_target_id = mode - 30
            self.is_paused = False
            self.mission_completed = False
            self.send_log(f'🚚 [이동 모드] {self.current_target_id}번 목적지로 이동을 시작합니다.')
            self.send_nav_goal(self.current_target_id)

    # [원상 복구] rear_person 토픽으로 사람 유무 판단
    def person_callback(self, msg):
        if 20 <= self.current_mode <= 27:
            is_person_present = msg.data 
            
            if not is_person_present and not self.is_paused:
                self.send_log('⚠️ 사람 놓침! 안내 주행 일시 정지.', 'warn')
                self.is_paused = True
                self.cancel_current_goal() 
                
            elif is_person_present and self.is_paused:
                self.send_log('▶️ 사람 재인식! 안내 주행 재개.')
                self.is_paused = False
                self.send_nav_goal(self.current_target_id) 

    def send_nav_goal(self, target_id):
        if target_id not in self.marker_database:
            self.send_log(f'❌ 알 수 없는 목적지 ID: {target_id}', 'error')
            return

        x, y, base_yaw = self.marker_database[target_id]
        
        target_yaw = base_yaw
        if 30 <= self.current_mode <= 37:
            target_yaw = base_yaw + math.pi

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(target_yaw / 2.0)

        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_current_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.cmd_vel_pub.publish(Twist())
        self.mission_completed = False

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.send_log('❌ Nav2 경로 생성이 거부되었습니다.', 'error')
            return
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4: 
            self.send_log('🏁 목적지에 안전하게 도착 및 정렬 완료했습니다.')
            self.mission_completed = True 

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Commander()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.cmd_vel_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()