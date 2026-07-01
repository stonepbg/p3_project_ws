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
        self.state = 'WAITING'  # WAITING -> CRAB_WALK -> ALIGNING -> DONE
        
        self.target_y = 0.0
        self.crab_start_time = 0.0
        self.crab_duration = 0.0
        self.crab_direction = 0.0
        
        self.get_logger().info('🧩 픽업 정밀 밀착 노드(Mode 51) 가동 준비 완료')

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
        
        # 1. 처음 좌표를 받았을 때 딱 1번만 실행 (ACK 핸드셰이크)
        if self.state == 'WAITING':
            self.target_y = msg.y
            self.send_log(f'🎯 목표 좌표 수신 (Y오차: {self.target_y:.2f}m). 52번 수신 확인 신호 발송.')
            
            # 통신 유실을 대비해 52번 ACK를 3연사
            for _ in range(3):
                ack_msg = Int32()
                ack_msg.data = 52
                self.ack_pub.publish(ack_msg)
            
            # 2. 측면(게걸음) 이동 시간 계산 (속도: 10cm/s 기준)
            speed_y = 0.1 
            self.crab_duration = abs(self.target_y) / speed_y
            self.crab_direction = 1.0 if self.target_y > 0 else -1.0
            
            self.crab_start_time = time.time()
            self.state = 'CRAB_WALK'

    def scan_callback(self, msg):
        if self.current_agv_mode != 51: return
        if self.state == 'WAITING' or self.state == 'DONE': return
        
        current_time = time.time()
        twist = Twist()
        
        # [1단계] 받은 Y좌표만큼 메카넘 휠로 게걸음 (측면 정렬)
        if self.state == 'CRAB_WALK':
            if current_time - self.crab_start_time < self.crab_duration:
                twist.linear.y = 0.1 * self.crab_direction
                self.cmd_vel_pub.publish(twist)
            else:
                self.send_log('🦀 측면 정렬 완료. 라이다 기반 전방 평행 밀착을 시작합니다.')
                self.state = 'ALIGNING'
                
        # [2단계] 라이다를 이용해 책상과 평행을 맞추며 10cm까지 전진
        elif self.state == 'ALIGNING':
            min_f = float('inf') # 정면
            min_l = float('inf') # 좌측 전방
            min_r = float('inf') # 우측 전방
            
            for i, r in enumerate(msg.ranges):
                if r < 0.05 or r > 2.0 or math.isinf(r) or math.isnan(r): continue
                angle = msg.angle_min + i * msg.angle_increment
                deg = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
                
                # 좁은 각도로 평면(책상) 측정
                if -5 <= deg <= 5: min_f = min(min_f, r)
                elif 15 <= deg <= 25: min_l = min(min_l, r)
                elif -25 <= deg <= -15: min_r = min(min_r, r)
                
            # 측정 실패 시 기본값 설정 (장애물 없음 간주)
            if math.isinf(min_f): min_f = 2.0
            if math.isinf(min_l): min_l = 2.0
            if math.isinf(min_r): min_r = 2.0
            
            # 좌우 오차 (평행 여부 확인)
            diff = min_l - min_r
            
            # (1) 각도 보정 (평행이 아니면 회전)
            # 좌측이 더 멀면 양수(diff>0) -> 왼쪽으로 회전(0.15)
            if abs(diff) > 0.02: 
                twist.angular.z = 0.15 if diff > 0 else -0.15
            
            # (2) 거리 보정 (10cm가 될 때까지 전진)
            if min_f > 0.10: 
                twist.linear.x = 0.05 # 5cm/s 아주 느린 속도로 접근
            else:
                twist.linear.x = 0.0
                
            # (3) 완료 조건: 오차가 3cm 이내로 평행하고, 정면이 딱 10.5cm 이하일 때
            if abs(diff) <= 0.03 and min_f <= 0.105:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                twist.linear.y = 0.0
                self.cmd_vel_pub.publish(twist)
                
                self.send_log('✅ 가판대 10cm 정밀 밀착 및 평행 정렬 완료! 로봇팔 가동 신호(1)를 전송합니다.')
                
                # FSM에 도착 신호 전송
                for _ in range(3):
                    status_msg = Int32()
                    status_msg.data = 1
                    self.status_pub.publish(status_msg)
                
                self.state = 'DONE'
                return
                
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