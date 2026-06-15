#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        
        # 1. 카메라 노드에서 보내는 데이터 구독 (/target_cmd)
        self.subscription = self.create_subscription(
            Twist,
            '/target_cmd',
            self.target_callback,
            10)
            
        # 2. 로봇 하부 모터로 주행 명령 발행 (/cmd_vel)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- 제어 파라미터 (튜닝값) ---
        self.target_distance = 1.0  # 타겟 1m 앞에서 정지
        
        # 선속도 제어 파라미터 (Linear)
        self.kp_linear = 0.5        # 전진 속도 비례 상수
        self.max_linear_speed = 0.4 # 최대 전진 속도 (m/s)
        self.deadband_linear = 0.1  # 1m +- 10cm 안에서는 멈춤 (진동 방지)
        
        # 각속도 제어 파라미터 (Angular)
        self.kp_angular = 1.2       # 회전 속도 비례 상수
        self.max_angular_speed = 0.8 # 최대 회전 속도 (rad/s)
        self.deadband_angular = 0.05 # +- 0.05 rad (약 2.8도) 안에서는 회전 정지
        
        self.get_logger().info('Person Follower Node has been started!')
        self.get_logger().info(f'Target Distance: {self.target_distance}m')

    def target_callback(self, msg):
        # 카메라 파트에서 받은 원시 데이터
        current_distance = msg.linear.x
        target_angle_rad = msg.angular.z
        
        cmd_msg = Twist()
        
        # 만약 타겟 소실 상태 (카메라 노드에서 거리를 0으로 보낼 경우 등) 예외 처리
        if current_distance <= 0.01:
            # 타겟을 찾기 위해 제자리에서 천천히 회전 (카메라 파트에서 이미 처리했다면 생략 가능)
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
            # 오차에 비례 상수 곱하기
            raw_linear_vel = distance_error * self.kp_linear
            # 최대 속도 제한 (클리핑)
            cmd_msg.linear.x = max(min(raw_linear_vel, self.max_linear_speed), -self.max_linear_speed)

        # ---------------------------------------------
        # 2. 회전 속도 계산 (P Control)
        # ---------------------------------------------
        # 약속한 부호(좌측+, 우측-)가 cmd_vel의 회전 방향과 일치하므로 그대로 사용
        if abs(target_angle_rad) < self.deadband_angular:
            cmd_msg.angular.z = 0.0
        else:
            raw_angular_vel = target_angle_rad * self.kp_angular
            cmd_msg.angular.z = max(min(raw_angular_vel, self.max_angular_speed), -self.max_angular_speed)

        # ---------------------------------------------
        # 3. 로봇으로 명령 전송
        # ---------------------------------------------
        self.publisher.publish(cmd_msg)
        
        # 현재 상태 로깅 (디버깅용)
        self.get_logger().debug(
            f'Dist: {current_distance:.2f}m (Err: {distance_error:.2f}) -> Vel: {cmd_msg.linear.x:.2f} | '
            f'Ang: {target_angle_rad:.2f}rad -> AngVel: {cmd_msg.angular.z:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 안전 종료 시 로봇 정지
        stop_msg = Twist()
        node.publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
