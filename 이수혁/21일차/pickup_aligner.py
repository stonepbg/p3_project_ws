
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
        self.stabilize_start_time = 0.0  
        self.wait_start_time = 0.0
        self.blind_search_start = 0.0
        self.request_time = 0.0
        self.last_log_time = time.time()
        
        self.current_map_yaw = None 
        self.turn_sign = 0.0         
        self.initial_turn_sign = 0.0         # [신규] 최초 회전 방향을 기억할 변수
        self.blind_search_dir = 0.0  
        
        # 타겟 추적용 변수
        self.target_seen = False             
        self.exact_target_received = False   
        self.current_msg_y = 0.0             
        self.last_target_rx_time = 0.0       
        
        self.get_logger().info('🧩 픽업 밀착 노드 가동 (최초 회전 방향 기억 로직 적용 완료)')

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
            self.state = 'APPROACH_50'
            self.approach_start_time = time.time()
            self.initial_turn_sign = 0.0  # [신규] 모드 시작 시 방향 기억 초기화
            self.send_log('🚀 51번 진입: 타겟 50cm 앞까지 직진합니다.')
        elif self.current_agv_mode != 51:
            self.state = 'WAITING'

    def target_callback(self, msg):
        if self.current_agv_mode != 51: return
        
        if (msg.x == 0.0 and msg.y == 0.0) or abs(msg.y) > 0.40:
            return 
            
        self.current_msg_y = msg.y
        self.last_target_rx_time = time.time()
        self.target_seen = True

        if self.state == 'WAITING_EXACT_TARGET':
            self.exact_target_received = True
            self.target_y = msg.y + 0.12
            self.send_log(f'🎯 정확한 정밀 좌표 수신 완료! (보정 Y: {self.target_y:.2f}m). 게걸음 시작.')
            
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

        # ----------------------------------------------------
        # [1단계] 50cm 접근
        # ----------------------------------------------------
        if self.state == 'APPROACH_50':
            if current_time - self.approach_start_time > 15.0:
                self.send_log('⚠️ 50cm 접근 타임아웃. 강제 정렬로 넘어갑니다.', 'warn')
                self.state = 'ALIGN_TO_STAND'
                self.align_start_time = current_time
                return
                
            if min_stop_dist <= 0.50:
                self.send_log('✅ 타겟 50cm 접근 완료. 가판대 180도 정렬을 시작합니다.')
                self.state = 'ALIGN_TO_STAND'
                self.align_start_time = current_time
                self.cmd_vel_pub.publish(Twist())
            else:
                twist.angular.z = 0.0  
                twist.linear.x = 0.05  
                self.cmd_vel_pub.publish(twist)
            return

        # ----------------------------------------------------
        # [2단계] AMCL 180도 정렬
        # ----------------------------------------------------
        elif self.state == 'ALIGN_TO_STAND':
            if self.current_map_yaw is None: return
            
            target_yaw = math.pi 
            error = target_yaw - self.current_map_yaw
            while error > math.pi: error -= 2.0 * math.pi
            while error < -math.pi: error += 2.0 * math.pi
            
            # [핵심] 가장 처음 판단한 큰 회전 방향을 영구 기록
            if self.initial_turn_sign == 0.0:
                self.initial_turn_sign = 1.0 if error > 0 else -1.0
            
            if abs(error) > 0.05:
                self.turn_sign = 1.0 if error > 0 else -1.0
                p_speed = abs(error) * 0.8
                speed = max(min(p_speed, 0.4), 0.18)
                
                twist.angular.z = self.turn_sign * speed
                self.cmd_vel_pub.publish(twist)
            else:
                # [수정] 마지막에 찔끔 흔들린 turn_sign이 아니라, 처음에 크게 돌았던 initial_turn_sign을 기준으로 탐색 방향 결정
                self.blind_search_dir = 1.0 if self.initial_turn_sign < 0 else -1.0
                self.send_log('📐 180도 정렬 달성. 기체를 1.5초간 완벽히 정지합니다.')
                self.state = 'ALIGN_STABILIZE'
                self.stabilize_start_time = current_time
                self.cmd_vel_pub.publish(Twist())
            return

        # ----------------------------------------------------
        # [3단계] 안정화 대기
        # ----------------------------------------------------
        elif self.state == 'ALIGN_STABILIZE':
            self.cmd_vel_pub.publish(Twist())  
            if current_time - self.stabilize_start_time > 1.5:
                self.send_log('✅ 안정화 완료! 시야에 진짜 블록이 있는지 2초간 확인합니다.')
                self.target_seen = False
                self.wait_start_time = current_time
                self.state = 'WAIT_FOR_FIRST_SIGHT'
            return

        # ----------------------------------------------------
        # [4단계] 첫 시야 확인 (보이면 바로 요청)
        # ----------------------------------------------------
        elif self.state == 'WAIT_FOR_FIRST_SIGHT':
            self.cmd_vel_pub.publish(Twist()) 
            is_target_fresh = (current_time - self.last_target_rx_time < 1.0)
            
            if self.target_seen and is_target_fresh:
                self.send_log('👀 화면에서 즉시 타겟 발견! 4번 신호를 요청합니다.')
                self.state = 'REQUEST_NEW_COORDINATE'
                return
                
            if current_time - self.wait_start_time > 2.0:
                dir_text = "왼쪽" if self.blind_search_dir > 0 else "오른쪽"
                self.send_log(f'⚠️ 시야에 타겟이 없습니다. [{dir_text}]으로 시야 탐색 게걸음을 시작합니다.', 'warn')
                self.state = 'BLIND_SEARCH'
                self.blind_search_start = current_time
            return

        # ----------------------------------------------------
        # [5단계] 시야 탐색 (게걸음으로 중앙까지 맞추기)
        # ----------------------------------------------------
        elif self.state == 'BLIND_SEARCH':
            if current_time - self.blind_search_start > 12.0:
                self.send_log('❌ 12초간 탐색했으나 타겟을 중앙에 맞추지 못해 강제 종료합니다.', 'error')
                self.state = 'DONE'
                self.cmd_vel_pub.publish(Twist())
                return
                
            is_target_fresh = (current_time - self.last_target_rx_time < 0.5)
            
            if self.target_seen and is_target_fresh:
                if abs(self.current_msg_y) <= 0.08:
                    self.cmd_vel_pub.publish(Twist()) 
                    self.send_log('🎯 탐색 중 타겟이 화면 중앙 근처에 들어왔습니다! 4번 신호를 요청합니다.')
                    self.state = 'REQUEST_NEW_COORDINATE'
                    return
                else:
                    current_dir = 1.0 if self.current_msg_y > 0 else -1.0
                    twist.linear.y = 0.06 * current_dir
                    
                    if current_time - self.last_log_time > 1.0:
                        dir_text = "왼쪽" if current_dir > 0 else "오른쪽"
                        self.send_log(f'👀 블록 발견! 화면 중앙에 맞추기 위해 [{dir_text}] 미세 이동 중... (오차: {self.current_msg_y:.2f}m)')
                        self.last_log_time = current_time
            else:
                twist.linear.y = 0.06 * self.blind_search_dir
                if current_time - self.last_log_time > 1.0:
                    dir_text = "왼쪽" if self.blind_search_dir > 0 else "오른쪽"
                    self.send_log(f'🔍 블록 탐색 게걸음 중... ({dir_text})')
                    self.last_log_time = current_time
                    
            self.cmd_vel_pub.publish(twist)
            return

        # ----------------------------------------------------
        # [6단계] 4번 신호 발송
        # ----------------------------------------------------
        elif self.state == 'REQUEST_NEW_COORDINATE':
            self.cmd_vel_pub.publish(Twist()) 
            self.send_log('🛑 확실한 정지 상태에서 4번 신호를 보내 정밀 좌표를 요청합니다.')
            
            for _ in range(3):
                req_msg = Int32()
                req_msg.data = 4
                self.status_pub.publish(req_msg)
            
            self.exact_target_received = False
            self.request_time = current_time
            self.state = 'WAITING_EXACT_TARGET'
            return

        # ----------------------------------------------------
        # [7단계] 정밀 좌표 대기
        # ----------------------------------------------------
        elif self.state == 'WAITING_EXACT_TARGET':
            self.cmd_vel_pub.publish(Twist()) 
            
            if current_time - self.request_time > 5.0:
                self.send_log('⚠️ 4번 신호 응답 없음 (오인식 가능성). 다시 시야 탐색 게걸음으로 돌아갑니다.', 'warn')
                self.target_seen = False
                self.state = 'BLIND_SEARCH'
                self.blind_search_start = current_time
            return

        # ----------------------------------------------------
        # [8단계] 정상 게걸음
        # ----------------------------------------------------
        elif self.state == 'CRAB_WALK':
            if current_time - self.crab_start_time < self.crab_duration:
                twist.linear.y = 0.1 * self.crab_direction
                self.cmd_vel_pub.publish(twist)
            else:
                self.send_log('🦀 게걸음 이동 완료. 전방 직진 밀착을 시작합니다.')
                self.state = 'ALIGNING'
            return
                
        # ----------------------------------------------------
        # [9단계] 직진 밀착
        # ----------------------------------------------------
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
