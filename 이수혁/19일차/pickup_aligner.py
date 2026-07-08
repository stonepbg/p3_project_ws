import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import time

class PickupAligner(Node):
    def __init__(self):
        super().__init__('pickup_aligner')
        
        self.mode_sub = self.create_subscription(Int32, '/internal_mode', self.mode_callback, 10)
        self.target_sub = self.create_subscription(Point, '/pickup_target', self.target_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, amcl_qos)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ack_pub = self.create_publisher(Int32, '/AGV_mode_ack', 10)
        self.status_pub = self.create_publisher(Int32, '/AGV_status', 10)
        self.log_pub = self.create_publisher(String, '/AGV_log', 10)
        
        self.current_agv_mode = 0
        self.state = 'WAITING' 
        
        self.target_y = 0.0
        self.crab_start_time = 0.0
        self.crab_duration = 0.0
        self.crab_direction = 0.0
        
        self.approach_start_time = 0.0
        self.align_start_time = 0.0
        self.wait_start_time = 0.0
        self.blind_search_start = 0.0
        self.last_log_time = time.time()
        
        self.current_map_yaw = None 
        self.turn_sign = 0.0         # 회전 방향 기억 (1.0: CCW 좌회전, -1.0: CW 우회전)
        self.blind_search_dir = 0.0  # 탐색 게걸음 방향 (1.0: 왼쪽, -1.0: 오른쪽)
        self.target_valid = False
        
        self.get_logger().info('🧩 픽업 밀착 노드 가동 (안전 필터 및 좌/우 지능형 시야 탐색 적용)')

    def send_log(self, text, level='info'):
        if level == 'info': self.get_logger().info(text)
        elif level == 'warn': self.get_logger().warn(text)
        elif level == 'error': self.get_logger().error(text)
        msg = String()
        msg.data = f"[픽업 밀착] {text}"
        self.log_pub.publish(msg)

    def amcl_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_map_yaw = math.atan2(siny_cosp, cosy_cosp)

    def mode_callback(self, msg):
        prev = self.current_agv_mode
        self.current_agv_mode = msg.data
        
        if self.current_agv_mode == 51 and prev != 51:
            self.state = 'APPROACH_40'
            self.approach_start_time = time.time()
            self.send_log('🚀 51번 진입: 현재 각도를 유지하며 타겟 40cm 앞까지 직진합니다.')
        elif self.current_agv_mode != 51:
            self.state = 'WAITING'

    def target_callback(self, msg):
        if self.current_agv_mode != 51: return
        
        # 엄격한 대기 상태이거나, 시야 탐색 중에 진짜 타겟이 들어왔을 때
        if self.state in ['WAITING_TARGET_STRICT', 'BLIND_SEARCH']:
            # [안전 필터] 오차가 40cm를 넘어가면 가짜(노이즈)로 간주하고 무시함
            if abs(msg.y) > 0.40:
                if time.time() - self.last_log_time > 1.0:
                    self.send_log(f'⚠️ 비정상 노이즈 타겟 무시 (오차: {msg.y:.2f}m)', 'warn')
                    self.last_log_time = time.time()
                return
                
            self.target_valid = True
            self.target_y = msg.y + 0.12
            self.send_log(f'🎯 진짜 타겟 포착! (오차: {msg.y:.2f}m -> 보정: {self.target_y:.2f}m). 정밀 게걸음 시작.')
            
            for _ in range(3):
                ack_msg = Int32()
                ack_msg.data = 52
                self.ack_pub.publish(ack_msg)
            
            speed_y = 0.1 
            self.crab_duration = abs(self.target_y) / speed_y
            self.crab_direction = 1.0 if self.target_y > 0 else -1.0
            
            self.crab_start_time = time.time()
            self.state = 'CRAB_WALK'

    def scan_callback(self, msg):
        if self.current_agv_mode != 51: return
        if self.state in ['WAITING', 'DONE']: return
        
        current_time = time.time()
        twist = Twist()
        
        min_stop_dist = float('inf') 
        for i, r in enumerate(msg.ranges):
            if r < 0.05 or r > 8.0 or math.isinf(r) or math.isnan(r): continue
            angle = msg.angle_min + i * msg.angle_increment
            raw_deg = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
            
            real_front_deg = raw_deg + 180.0
            if real_front_deg > 180.0: real_front_deg -= 360.0

            if -25 <= real_front_deg <= 25:
                min_stop_dist = min(min_stop_dist, r)
            
        if math.isinf(min_stop_dist): min_stop_dist = 9.99

        # [1단계] 40cm 접근
        if self.state == 'APPROACH_40':
            if current_time - self.approach_start_time > 15.0:
                self.send_log('⚠️ 40cm 접근 타임아웃. 강제 정렬로 넘어갑니다.', 'warn')
                self.state = 'ALIGN_TO_STAND'
                self.align_start_time = current_time
                return
                
            if min_stop_dist <= 0.40:
                self.send_log('✅ 타겟 40cm 접근 완료. 가판대 180도 정렬을 시작합니다.')
                self.state = 'ALIGN_TO_STAND'
                self.align_start_time = current_time
                self.cmd_vel_pub.publish(Twist())
            else:
                twist.angular.z = 0.0  
                twist.linear.x = 0.05  
                self.cmd_vel_pub.publish(twist)
            return

        # [2단계] AMCL 맵 좌표 기반 정렬 및 회전 방향 기억
        elif self.state == 'ALIGN_TO_STAND':
            if self.current_map_yaw is None: return
            
            target_yaw = math.pi 
            error = target_yaw - self.current_map_yaw
            while error > math.pi: error -= 2.0 * math.pi
            while error < -math.pi: error += 2.0 * math.pi
            
            if abs(error) > 0.05:
                self.turn_sign = 1.0 if error > 0 else -1.0 # CCW면 양수, CW면 음수 저장
                p_speed = abs(error) * 0.8
                speed = max(min(p_speed, 0.4), 0.25)
                twist.angular.z = self.turn_sign * speed
                self.cmd_vel_pub.publish(twist)
                
                if current_time - self.last_log_time > 0.5:
                    self.send_log(f'🔄 180도 맞추는 중... (오차: {math.degrees(error):.1f}도)')
                    self.last_log_time = current_time
            else:
                # 회전이 끝났을 때, CW로 돌았다면 왼쪽(+1) 탐색, CCW로 돌았다면 오른쪽(-1) 탐색 설정
                self.blind_search_dir = 1.0 if self.turn_sign < 0 else -1.0
                
                self.send_log('📐 정렬 완료! 좌표 요청(4번) 신호를 보내고 2초 대기합니다.')
                for _ in range(3):
                    req_msg = Int32()
                    req_msg.data = 4
                    self.status_pub.publish(req_msg)
                
                self.target_valid = False
                self.wait_start_time = current_time
                self.state = 'WAITING_TARGET_STRICT'
                self.cmd_vel_pub.publish(Twist())
            return

        # [3단계] 타겟 응답 대기 (2초)
        elif self.state == 'WAITING_TARGET_STRICT':
            if current_time - self.wait_start_time > 2.0:
                if not self.target_valid:
                    dir_text = "왼쪽" if self.blind_search_dir > 0 else "오른쪽"
                    self.send_log(f'⚠️ 타겟 유실! 회전 방향을 역산하여 [{dir_text}]으로 시야 탐색을 시작합니다.', 'warn')
                    self.state = 'BLIND_SEARCH'
                    self.blind_search_start = current_time
            self.cmd_vel_pub.publish(Twist()) # 제자리 대기
            return

        # [4단계] 지능형 시야 탐색 (게걸음)
        elif self.state == 'BLIND_SEARCH':
            if current_time - self.blind_search_start > 8.0:
                self.send_log('❌ 8초간 탐색했으나 타겟을 찾지 못해 강제 종료합니다.', 'error')
                self.state = 'DONE'
                self.cmd_vel_pub.publish(Twist())
                return
                
            # target_callback 에서 올바른 타겟이 들어오면 자동으로 CRAB_WALK 로 넘어감
            twist.linear.y = 0.05 * self.blind_search_dir
            self.cmd_vel_pub.publish(twist)
            
            if current_time - self.last_log_time > 1.0:
                dir_text = "왼쪽" if self.blind_search_dir > 0 else "오른쪽"
                self.send_log(f'👀 블록 탐색 게걸음 중... ({dir_text})')
                self.last_log_time = current_time
            return

        # [5단계] 정상 게걸음
        elif self.state == 'CRAB_WALK':
            if current_time - self.crab_start_time < self.crab_duration:
                twist.linear.y = 0.1 * self.crab_direction
                self.cmd_vel_pub.publish(twist)
            else:
                self.send_log('🦀 게걸음 이동 완료. 전방 직진 밀착을 시작합니다.')
                self.state = 'ALIGNING'
            return
                
        # [6단계] 직진 밀착
        elif self.state == 'ALIGNING':
            if min_stop_dist <= 0.22:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
                self.send_log('✅ 타겟 22cm 도달 완료! 즉시 멈추고 1번 신호를 발송합니다.')
                for _ in range(3):
                    status_msg = Int32()
                    status_msg.data = 1
                    self.status_pub.publish(status_msg)
                
                self.state = 'DONE'
                return
            else:
                twist.angular.z = 0.0  
                twist.linear.x = 0.05  
                self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PickupAligner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()
