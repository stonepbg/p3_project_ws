import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped
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
        
        # [핵심 추가] 맵 상의 절대 내 위치(AMCL)를 받아오기 위한 구독
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        
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
        
        self.current_map_yaw = None # 현재 맵 상의 내 절대 각도
        
        self.get_logger().info('🧩 픽업 밀착 노드 가동 (AMCL 맵 좌표 기반 순간이동 방지 및 절대 정면 정렬 적용)')

    def send_log(self, text, level='info'):
        if level == 'info': self.get_logger().info(text)
        elif level == 'warn': self.get_logger().warn(text)
        msg = String()
        msg.data = f"[픽업 밀착] {text}"
        self.log_pub.publish(msg)

    def amcl_callback(self, msg):
        # 쿼터니언 데이터를 사람이 읽을 수 있는 라디안(yaw) 각도로 변환
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_map_yaw = math.atan2(siny_cosp, cosy_cosp)

    def mode_callback(self, msg):
        prev = self.current_agv_mode
        self.current_agv_mode = msg.data
        
        if self.current_agv_mode == 51 and prev != 51:
            self.state = 'ALIGN_TO_STAND'
            self.align_start_time = time.time()
            self.send_log('🔄 51번 진입: 맵 좌표(AMCL)를 이용해 2,3,4,5번 반대편 가판대(서쪽, 180도)로 정렬합니다.')
        elif self.current_agv_mode != 51:
            self.state = 'WAITING'

    def target_callback(self, msg):
        if self.current_agv_mode != 51: return
        
        if self.state == 'WAITING_TARGET':
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
        # [1단계] AMCL 맵 좌표 기반 절대 정렬 (라이다 노이즈 무시)
        # ----------------------------------------------------
        if self.state == 'ALIGN_TO_STAND':
            if self.current_map_yaw is None:
                if current_time - self.last_log_time > 1.0:
                    self.send_log('⏳ 맵 위치(AMCL) 데이터를 기다리는 중입니다...')
                    self.last_log_time = current_time
                return
            
            # 타겟 각도: 서쪽 (180도 = math.pi). 2,3,4,5번 마커의 반대편.
            target_yaw = math.pi 
            error = target_yaw - self.current_map_yaw
            
            # 오차를 -180도 ~ 180도(-pi ~ pi) 사이로 정규화
            while error > math.pi: error -= 2.0 * math.pi
            while error < -math.pi: error += 2.0 * math.pi
            
            # 무한 대기 방지
            if current_time - self.align_start_time > 8.0:
                self.send_log('⚠️ 맵 기반 자세 교정 시간 초과. 타겟 탐색을 대기합니다.', 'warn')
                self.state = 'WAITING_TARGET'
                self.cmd_vel_pub.publish(Twist())
                return
                
            # 오차가 약 2.8도(0.05 rad) 이내가 될 때까지 부드럽게 P 제어 회전
            if abs(error) > 0.05:
                # 휠 슬립과 순간이동(AMCL 튐)을 막기 위해 최대 속도를 0.3으로 부드럽게 제한
                twist.angular.z = max(min(error * 0.8, 0.3), -0.3)
                self.cmd_vel_pub.publish(twist)
                
                if current_time - self.last_log_time > 0.5:
                    self.send_log(f'🔄 맵 좌표 기반 정렬 중... (현재: {math.degrees(self.current_map_yaw):.1f}도, 오차: {math.degrees(error):.1f}도)')
                    self.last_log_time = current_time
            else:
                self.send_log('📐 가판대 정면(180도)으로 맵 기반 정렬 완료! 타겟 탐색 대기중.')
                self.state = 'WAITING_TARGET'
                self.cmd_vel_pub.publish(Twist())
            return

        # ----------------------------------------------------
        # 공통 라이다 데이터 파싱 (ALIGNING 직진 밀착 단계 전용)
        # ----------------------------------------------------
        min_stop_dist = float('inf') 
        min_l = float('inf') 
        min_r = float('inf') 
        
        for i, r in enumerate(msg.ranges):
            if r < 0.05 or r > 8.0 or math.isinf(r) or math.isnan(r): continue
            angle = msg.angle_min + i * msg.angle_increment
            raw_deg = math.degrees(math.atan2(math.sin(angle), math.cos(angle)))
            
            real_front_deg = raw_deg + 180.0
            if real_front_deg > 180.0:
                real_front_deg -= 360.0

            if -25 <= real_front_deg <= 25:
                min_stop_dist = min(min_stop_dist, r)
            # 전진할 때는 이미 책상을 마주보고 있으므로 라이다 비교가 매우 안정적임
            if 15 <= real_front_deg <= 45: min_l = min(min_l, r)
            elif -45 <= real_front_deg <= -15: min_r = min(min_r, r)
            
        if math.isinf(min_stop_dist): min_stop_dist = 9.99
        if math.isinf(min_l): min_l = 9.99
        if math.isinf(min_r): min_r = 9.99
        
        diff = 0.0
        if min_l != 9.99 and min_r != 9.99:
            diff = min_l - min_r

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
                
            elif abs(diff) > 0.025 and min_l != 9.99 and min_r != 9.99:
                twist.linear.x = 0.0  
                twist.angular.z = 0.15 if diff > 0 else -0.15
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
