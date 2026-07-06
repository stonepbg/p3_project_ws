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
        self.pre_align_start_time = 0.0
        
        self.last_log_time = time.time()
        
        self.get_logger().info('🧩 픽업 밀착 노드 가동 (사전 자세 교정 기능 추가 / 물리적 전방 22cm 정지 적용)')

    def send_log(self, text, level='info'):
        if level == 'info': self.get_logger().info(text)
        elif level == 'warn': self.get_logger().warn(text)
        msg = String()
        msg.data = f"[픽업 밀착] {text}"
        self.log_pub.publish(msg)

    def mode_callback(self, msg):
        self.current_agv_mode = msg.data
        if self.current_agv_mode != 51:
            self.state = 'WAITING'

    def target_callback(self, msg):
        if self.current_agv_mode != 51: return
        
        if self.state == 'WAITING':
            # 실제 목적지보다 왼쪽으로 10cm(0.10m) 더 가도록 Y 좌표 보정 (+가 왼쪽)
            self.target_y = msg.y + 0.10
            
            self.send_log(f'🎯 목표 좌표 수신 (원본: {msg.y:.2f}m -> 보정 Y오차: {self.target_y:.2f}m). 52번 신호 발송.')
            
            for _ in range(3):
                ack_msg = Int32()
                ack_msg.data = 52
                self.ack_pub.publish(ack_msg)
            
            speed_y = 0.1 
            self.crab_duration = abs(self.target_y) / speed_y
            self.crab_direction = 1.0 if self.target_y > 0 else -1.0
            
            # [수정] 바로 게걸음을 시작하지 않고, 자세 교정 단계(PRE_ALIGN)로 먼저 진입합니다.
            self.pre_align_start_time = time.time()
            self.state = 'PRE_ALIGN'
            self.send_log('📐 [1단계] 라이다를 이용해 전방 책상(벽)과 평행하게 자세를 교정합니다.')

    def scan_callback(self, msg):
        if self.current_agv_mode != 51: return
        if self.state == 'WAITING' or self.state == 'DONE': return
        
        current_time = time.time()
        twist = Twist()
        
        min_stop_dist = float('inf') 
        min_l = float('inf') 
        min_r = float('inf') 
        
        for i, r in enumerate(msg.ranges):
            if r < 0.05 or r > 8.0 or math.isinf(r) or math.isnan(r): continue
            angle = msg.angle_min + i * msg.angle_increment
            deg = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
            
            # 물리적 전방(코드상 180도 및 -180도 부근)을 정지용으로 확인
            if deg >= 155 or deg <= -155: 
                min_stop_dist = min(min_stop_dist, r)
                
            # 평행 정렬용 (기존에 잘 작동하던 코드 그대로 유지)
            if 15 <= deg <= 45: min_l = min(min_l, r)
            elif -45 <= deg <= -15: min_r = min(min_r, r)
            
        if math.isinf(min_stop_dist): min_stop_dist = 9.99
        if math.isinf(min_l): min_l = 9.99
        if math.isinf(min_r): min_r = 9.99
        
        diff = 0.0
        if min_l != 9.99 and min_r != 9.99:
            diff = min_l - min_r
        
        if current_time - self.last_log_time > 0.5:
            self.send_log(f'🔍 [{self.state}] 정지거리: {min_stop_dist:.2f}m | 좌: {min_l:.2f}m | 우: {min_r:.2f}m | 평행오차: {abs(diff):.2f}m')
            self.last_log_time = current_time
        
        # ----------------------------------------------------
        # 3단계 상태 머신 제어 로직
        # ----------------------------------------------------
        
        # [1단계] 제자리 회전으로 자세 평행 맞추기
        if self.state == 'PRE_ALIGN':
            # 무한 대기 방지: 4초가 넘어가면 정렬을 포기하고 바로 게걸음으로 넘어감
            if current_time - self.pre_align_start_time > 4.0:
                self.send_log('⏱️ 교정 시간 초과. [2단계] 메카넘 게걸음을 시작합니다.')
                self.state = 'CRAB_WALK'
                self.crab_start_time = current_time
            elif min_l != 9.99 and min_r != 9.99:
                if abs(diff) > 0.025:  # 좌우 오차가 2.5cm 이상이면 제자리 회전
                    twist.angular.z = 0.15 if diff > 0 else -0.15
                else:
                    self.send_log('✅ 자세 교정 완료! [2단계] 메카넘 게걸음을 시작합니다.')
                    self.state = 'CRAB_WALK'
                    self.crab_start_time = current_time
            else:
                self.send_log('⚠️ 라이다 벽면 미검출로 자세 교정 생략. [2단계] 게걸음 시작.')
                self.state = 'CRAB_WALK'
                self.crab_start_time = current_time
                
            self.cmd_vel_pub.publish(twist)

        # [2단계] 자세가 바르게 잡힌 상태에서 옆으로 이동
        elif self.state == 'CRAB_WALK':
            if current_time - self.crab_start_time < self.crab_duration:
                twist.linear.y = 0.1 * self.crab_direction
                self.cmd_vel_pub.publish(twist)
            else:
                self.send_log('🦀 게걸음 완료. [3단계] 전방 정밀 밀착을 시작합니다.')
                self.state = 'APPROACH'
                
        # [3단계] 정면으로 전진하며 22cm 도달 시 종료
        elif self.state == 'APPROACH':
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
                
            # 전진 중에도 각도가 2.5cm 이상 틀어지면 직진 멈추고 제자리 회전 미세 보정
            elif abs(diff) > 0.025 and min_l != 9.99 and min_r != 9.99:
                twist.linear.x = 0.0  
                twist.angular.z = 0.15 if diff > 0 else -0.15
                
            # 각도가 맞았으면 전진
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