#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math
import time

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        
        # 1. 카메라 노드에서 보내는 데이터 구독 (/target_cmd)
        self.subscription = self.create_subscription(
            Twist,
            '/target_cmd',
            self.target_callback,
            10)
            
        # [추가] 라이다 데이터 구독 (/scan) - QoS 프로파일 필수 적용
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data)
            
        # 2. 로봇 하부 모터로 주행 명령 발행 (/cmd_vel)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- 제어 파라미터 (튜닝값) ---
        self.target_distance = 1.0  # 타겟 1m 앞에서 정지
        self.safe_distance = 0.35   # [추가] 벽 충돌 방지 안전 거리 (0.35m)
        
        # 선속도 제어 파라미터 (Linear)
        self.kp_linear = 0.5        # 전진 속도 비례 상수
        self.max_linear_speed = 0.4 # 최대 전진 속도 (m/s)
        self.deadband_linear = 0.1  # 1m +- 10cm 안에서는 멈춤 (진동 방지)
        
        # 각속도 제어 파라미터 (Angular)
        self.kp_angular = 1.2       # 회전 속도 비례 상수
        self.max_angular_speed = 0.8 # 최대 회전 속도 (rad/s)
        self.deadband_angular = 0.05 # +- 0.05 rad (약 2.8도) 안에서는 회전 정지
        
        # [추가] 라이다 센서 데이터 저장 변수
        self.min_dist_front = float('inf')
        self.min_dist_back = float('inf')

        self.get_logger().info('Person Follower Node (LiDAR Safety & E-Stop applied) has been started!')
        self.get_logger().info(f'Target Distance: {self.target_distance}m')

    def scan_callback(self, msg):
        min_f = float('inf')
        min_b = float('inf')

        for i, r in enumerate(msg.ranges):
            # 노이즈 및 에러 데이터 필터링
            if r < 0.05 or r > 10.0 or math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            deg = math.degrees(angle)

            # 전방 60도(-30~30) 및 후방 60도(150~-150) 최소 거리 측정
            if -30 <= deg <= 30:
                min_f = min(min_f, r)
            elif deg >= 150 or deg <= -150:
                min_b = min(min_b, r)

        self.min_dist_front = min_f
        self.min_dist_back = min_b

    def target_callback(self, msg):
        # 카메라 파트에서 받은 원시 데이터
        current_distance = msg.linear.x
        target_angle_rad = msg.angular.z
        
        cmd_msg = Twist()
        
        # 만약 타겟 소실 상태 (카메라 노드에서 거리를 0으로 보낼 경우 등) 예외 처리
        if current_distance <= 0.01:
            self.get_logger().info('Target lost. Waiting or searching...')
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0 # 일단 정지
            self.publisher.publish(cmd_msg)
            return

        # ---------------------------------------------
        # 1. 전진/후진 속도 계산 (P Control)
        # ---------------------------------------------
        distance_error = current_distance - self.target_distance
        
        # 오차가 데드밴드(0.1m) 이내라면 선속도 0
        if abs(distance_error) < self.deadband_linear:
            cmd_msg.linear.x = 0.0
        else:
            raw_linear_vel = distance_error * self.kp_linear
            cmd_msg.linear.x = max(min(raw_linear_vel, self.max_linear_speed), -self.max_linear_speed)

        # ---------------------------------------------
        # 2. 회전 속도 계산 (P Control)
        # ---------------------------------------------
        if abs(target_angle_rad) < self.deadband_angular:
            cmd_msg.angular.z = 0.0
        else:
            raw_angular_vel = target_angle_rad * self.kp_angular
            cmd_msg.angular.z = max(min(raw_angular_vel, self.max_angular_speed), -self.max_angular_speed)

        # ---------------------------------------------
        # 3. [추가] 라이다 충돌 방지 안전장치
        # ---------------------------------------------
        status_text = "TRACKING"
        if cmd_msg.linear.x > 0 and self.min_dist_front < self.safe_distance:
            cmd_msg.linear.x = 0.0
            status_text = "FRONT_BLOCKED"
        elif cmd_msg.linear.x < 0 and self.min_dist_back < self.safe_distance:
            cmd_msg.linear.x = 0.0
            status_text = "BACK_BLOCKED"

        # ---------------------------------------------
        # 4. 로봇으로 명령 전송
        # ---------------------------------------------
        self.publisher.publish(cmd_msg)
        
        # 현재 상태 로깅 (화면에 보이도록 info로 수정)
        self.get_logger().info(
            f'[{status_text}] Dist: {current_distance:.2f}m (Err: {distance_error:.2f}) -> Vel: {cmd_msg.linear.x:.2f} | '
            f'Ang: {target_angle_rad:.2f}rad -> AngVel: {cmd_msg.angular.z:.2f} | '
            f'Lidar F: {self.min_dist_front:.2f}m, B: {self.min_dist_back:.2f}m'
        )

    # [추가] 긴급 정지 발동 함수
    def emergency_stop(self):
        self.get_logger().warn('EMERGENCY STOP: Halting all motors...')
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)
        time.sleep(0.1)  # 통신선이 끊어지기 전 모터가 명령을 받을 수 있도록 0.1초 딜레이 부여

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 안전 종료 시 무조건 긴급 정지 함수 실행 후 노드 종료
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
