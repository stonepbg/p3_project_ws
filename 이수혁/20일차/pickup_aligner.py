import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String
from rclpy.qos import qos_profile_sensor_data
import math
import time

class PickupAligner(Node):
    def __init__(self):
        super().__init__('pickup_aligner')
        
        self.mode_sub = self.create_subscription(Int32, '/internal_mode', self.mode_callback, 10)
        self.target_sub = self.create_subscription(Point, '/pickup_target', self.target_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
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
        
        self.align_start_time = 0.0
        self.last_log_time = time.time()
        
        self.get_logger().info('🧩 픽업 밀착 노드 가동 (좌우 라이다 밸런싱 기반 부드러운 사전 정면 정렬 적용)')

    def send_log(self, text, level='info'):
        if level == 'info': self.get_logger().info(text)
        elif level == 'warn': self.get_logger().warn(text)
        msg = String()
        msg.data = f"[픽업 밀착] {text}"
        self.log_pub.publish(msg)

    def mode_callback(self, msg):
        prev = self.current_agv_mode
        self.current_agv_mode = msg.data
        
        # 51번 모드 진입 시 불안정한 노이즈 탐색 대신, 부드러운 양측 밸런스 정렬 시작
        if self.current_agv_mode == 51 and prev != 51:
            self.state = 'ALIGN_TO_STAND'
            self.align_start_time = time.time()
            self.send_log('🔄 51번 모드 진입: 라이다 좌/우 거리를 비교하여 가판대 정면으로 부드럽게 회전합니다.')
        elif self.current_agv_mode != 51:
            self.state = 'WAITING'

    def target_callback(self, msg):
        if self.current_agv_mode != 51: return
        
        if self.state == 'WAITING_TARGET':
            # 실제 목적지보다 왼쪽으로 12cm 더 가도록 Y 좌표 보정
            self.target_y = msg.y + 0.12
            
            self.send_log(f'🎯 목표 좌표 수신 (원본: {msg.y:.2f}m -> 보정 Y오차: {self.target_y:.2f}m). 52번 신호 발송.')
            
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
        if self.state in ['WAITING', 'WAITING_TARGET', 'DONE']: return
        
        current_time = time.time()
        twist = Twist()
        
        # ----------------------------------------------------
        # 공통 라이다 데이터 파싱 (루프 상단에서 한 번만 처리하여 최적화)
        # ----------------------------------------------------
        min_stop_dist = float('inf') 
        min_l = float('inf') 
        min_r = float('inf') 
        
        for i, r in enumerate(msg.ranges):
            if r < 0.05 or r > 8.0 or math.isinf(r) or math.isnan(r): continue
            angle = msg.angle_min + i * msg.angle_increment
            deg = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
            
            # 후면 충돌 방지용 (물리적 전방)
            if deg >= 155 or deg <= -155: 
                min_stop_dist = min(min_stop_dist, r)
            # 좌우 대각선 평행 비교용 (안정적인 데이터 구간)
            if 15 <= deg <= 45: min_l = min(min_l, r)
            elif -45 <= deg <= -15: min_r = min(min_r, r)
            
        if math.isinf(min_stop_dist): min_stop_dist = 9.99
        if math.isinf(min_l): min_l = 9.99
        if math.isinf(min_r): min_r = 9.99
        
        diff = 0.0
        if min_l != 9.99 and min_r != 9.99:
            diff = min_l - min_r

        # ----------------------------------------------------
        # [1단계] 사전 정면 정렬 (최단거리 대신, 좌우 밸런스 맞추기)
        # ----------------------------------------------------
        if self.state == 'ALIGN_TO_STAND':
            # 무한 대기 방지 (5초 초과 시 타겟 탐색으로 강제 전환)
            if current_time - self.align_start_time > 5.0:
                self.send_log('⚠️ 자세 교정 시간 초과. 타겟 탐색을 대기합니다.', 'warn')
                self.state = 'WAITING_TARGET'
                self.cmd_vel_pub.publish(Twist())
                return
                
            # 좌우 거리 오차가 2.5cm 이내가 될 때까지 부드럽게 제자리 회전
            if min_l != 9.99 and min_r != 9.99:
                if abs(diff) > 0.025:
                    # 미끄러짐 방지를 위해 속도를 0.15 rad/s로 낮춤
                    twist.angular.z = 0.15 if diff > 0 else -0.15
                    self.cmd_vel_pub.publish(twist)
                    
                    if current_time - self.last_log_time > 0.5:
                        self.send_log(f'🔄 정면 각도 부드럽게 교정 중... (좌우 오차: {abs(diff):.3f}m)')
                        self.last_log_time = current_time
                else:
                    self.send_log('📐 가판대 수직 정면 회전 완료! 타겟 탐색 대기중.')
                    self.state = 'WAITING_TARGET'
                    self.cmd_vel_pub.publish(Twist())
            else:
                # 좌우 데이터가 충분하지 않으면 회전을 생략
                self.state = 'WAITING_TARGET'
                self.cmd_vel_pub.publish(Twist())
            return

        # ----------------------------------------------------
        # [2단계] 메카넘 게걸음 (12cm 보정 이동)
        # ----------------------------------------------------
        if self.state == 'CRAB_WALK':
            if current_time - self.crab_start_time < self.crab_duration:
                twist.linear.y = 0.1 * self.crab_direction
                self.cmd_vel_pub.publish(twist)
            else:
                self.send_log('🦀 측면 이동 완료. 전방 평행 밀착을 시작합니다.')
                self.state = 'ALIGNING'
            return
                
        # ----------------------------------------------------
        # [3단계] 전진 밀착 로직 (물리적 전방 22cm 정지)
        # ----------------------------------------------------
        elif self.state == 'ALIGNING':
            if current_time - self.last_log_time > 0.5:
                self.send_log(f'🔍 [라이다] 정지최단거리: {min_stop_dist:.2f}m | 평행오차: {abs(diff):.2f}m')
                self.last_log_time = current_time
            
            # 정지거리가 22cm 이하가 되면 무조건 즉시 종료
            if min_stop_dist <= 0.22:
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                
                self.send_log('✅ 타겟 도달 완료! 즉시 멈추고 1번 신호를 발송합니다.')
                for _ in range(3):
                    status_msg = Int32()
                    status_msg.data = 1
                    self.status_pub.publish(status_msg)
                
                self.state = 'DONE'
                return
                
            # 전진 중 각도가 틀어지면 직진 멈추고 제자리 회전 보정
            elif abs(diff) > 0.025 and min_l != 9.99 and min_r != 9.99:
                twist.linear.x = 0.0  
                twist.angular.z = 0.15 if diff > 0 else -0.15
                
            # 각도가 잘 맞았으면 전진
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
