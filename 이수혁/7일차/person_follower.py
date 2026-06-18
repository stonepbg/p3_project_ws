import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int32  # Int32 추가 (모드 수신용)
from rclpy.qos import qos_profile_sensor_data
import math
import time

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        
        # 외부 FSM으로부터 모드 수신 (10번일 때만 활성화)
        self.mode_sub = self.create_subscription(Int32, '/agv_mode', self.mode_callback, 10)
        self.current_agv_mode = 0 # 초기 상태는 대기
        
        self.subscription = self.create_subscription(Twist, '/target_cmd', self.target_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        # 제어 명령 퍼블리셔 (AGV 바퀴 직접 제어)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 대시보드 로그 전송용 퍼블리셔
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
        self.get_logger().info('Person Follower Node started! (Waiting for AGV Mode 10...)')

    def mode_callback(self, msg):
        prev_mode = self.current_agv_mode
        self.current_agv_mode = msg.data

        # 10번 모드로 새로 진입할 때
        if self.current_agv_mode == 10 and prev_mode != 10:
            self.get_logger().info('▶️ 10번 모드 진