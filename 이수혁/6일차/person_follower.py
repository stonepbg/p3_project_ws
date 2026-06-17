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
        
        # 1. 통합된 타겟 명령 구독 (/target_cmd)
        self.subscription = self.create_subscription(
            Twist,
            '/target_cmd',
            self.target_callback,
            10)
            
        # 2. 라이다 데이터 구독 (/scan)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data)
            
        # 3. 로봇 하부 모터로 주행 명령 발행 (/cmd_vel)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- 제어 및 안전 파라미터 ---
        self.target_distance = 1.0   # 목표 유지 거리 (1m)
        self.safe_distance = 0.35    # 라이다 충돌 방지 안전 거리 (0.35m)
        
        # 선속도 제어 (추종 모드)
        self.kp_linear = 0.5
        self.max_linear_speed = 0.4
        self.deadband_linear = 0.1
        
        # 각속도 제어 (추종 모드)
        self.kp_angular = 1.2
        self.max_angular_speed = 0.8
        self.deadband_angular = 0.05
        
        # 탐색 모드 파라미터 (10도씩 끊어 돌기 설정)
        self.search_speed = 0.3       # 탐색 회전 속도 (rad/s)
        self.step_angle_deg = 10.0    # 한 번에 회전할 각도
        self.step_angle_rad = math.radians(self.step_angle_deg)
        self.search_rotate_duration = self.step_angle_rad / self.search_speed # 10도 회전하는데 걸리는 시간
        
        # 상태 관리 및 내부 변수
        self.min_dist_front = float('inf')
        self.min_dist_back = float('inf')
        
        # Watchdog 및 로그 주기 타이머용 변수
        self.last_msg_time = self.get_clock().now()
        self.last_log_time = self.get_clock().now()
        self.current_mode_text = "INIT"
        
        # 탐색 제어 변수
        self.is_searching = False
        self.search_start_time = 0.0
        self.search_direction = 0.0 # 1.0: 좌회전, -1.0: 우회전
        
        # Watchdog 상시 감시 타이머 (10Hz, 0.1초 주기)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)

        self.get_logger().info('Integrated Person Follower Node with Watchdog has started!')

    def scan_callback(self, msg):
        min_f = float('inf')
        min_b = float('inf')

        for i, r in enumerate(msg.ranges):
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
        # 메시지 수신 시간 업데이트 (Watchdog 리셋)
        self.last_msg_time = self.get_clock().now()
        
        mode = msg.linear.z
        current_distance = msg.linear.x
        target_angle_rad = msg.angular.z
        
        cmd_msg = Twist()
        status_text = "TRACKING"

        # ---------------------------------------------
        # Mode 0.0 : 기존 추종 모드 (P 제어 + 라이다 안전장치)
        # ---------------------------------------------
        if mode == 0.0:
            self.is_searching = False # 탐색 상태 해제
            
            # 1. 선속도 계산
            distance_error = current_distance - self.target_distance
            if abs(distance_error) < self.deadband_linear:
                cmd_msg.linear.x = 0.0
            else:
                raw_linear_vel = distance_error * self.kp_linear
                cmd_msg.linear.x = max(min(raw_linear_vel, self.max_linear_speed), -self.max_linear_speed)

            # 2. 각속도 계산
            if abs(target_angle_rad) < self.deadband_angular:
                cmd_msg.angular.z = 0.0
            else:
                raw_angular_vel = target_angle_rad * self.kp_angular
                cmd_msg.angular.z = max(min(raw_angular_vel, self.max_angular_speed), -self.max_angular_speed)

            # 3. 라이다 충돌 방지 충돌 제어 적용
            if cmd_msg.linear.x > 0 and self.min_dist_front < self.safe_distance:
                cmd_msg.linear.x = 0.0
                status_text = "FRONT_BLOCKED"
            elif cmd_msg.linear.x < 0 and self.min_dist_back < self.safe_distance:
                cmd_msg.linear.x = 0.0
                status_text = "BACK_BLOCKED"

        # ---------------------------------------------
        # Mode 1.0 / 2.0 : 탐색 모드 (천천히 10도씩 끊어서 회전)
        # ---------------------------------------------
        elif mode == 1.0 or mode == 2.0:
            status_text = "SEARCH_LEFT" if mode == 1.0 else "SEARCH_RIGHT"
            direction = 1.0 if mode == 1.0 else -1.0
            
            current_time = time.time()
            
            # 새로운 탐색 명령이거나, 10도 회전 주기(회전+정지 대기)가 완전히 끝났을 때 새로 시작
            # 총 주기 = 10도 회전 시간 + 0.5초 정지 대기 (카메라가 사람을 인식할 시간 확보)
            total_step_duration = self.search_rotate_duration + 0.5 
            
            if not self.is_searching or (current_time - self.search_start_time > total_step_duration):
                self.is_searching = True
                self.search_start_time = current_time
                self.search_direction = direction

            elapsed_time = current_time - self.search_start_time
            
            # 계산된 회전 시간 동안만 모터를 돌리고, 남은 시간(0.5초)은 정지하여 카메라 탐색 유도
            if elapsed_time < self.search_rotate_duration:
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = self.search_direction * self.search_speed
                status_text += "_ROTATING"
            else:
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                status_text += "_WAITING"

        # ---------------------------------------------
        # Mode 3.0 : 발견 및 즉시 정지 (3초간 유지)
        # ---------------------------------------------
        elif mode == 3.0:
            self.is_searching = False
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            status_text = "FIND_STOP"

        # 최종 제어 명령 발행
        self.publisher.publish(cmd_msg)
        self.current_mode_text = status_text
        
        # 0.5초 주기로 터미널에 상태 출력 로그 (가시성 강화)
        self.print_clean_log(mode, current_distance, target_angle_rad, cmd_msg)

    def watchdog_check(self):
        """0.5초 동안 /target_cmd 토픽이 오지 않으면 안전을 위해 로봇을 멈춥니다."""
        now = self.get_clock().now()
        time_diff = (now - self.last_msg_time).nanoseconds / 1e9
        
        if time_diff > 0.5:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.publisher.publish(stop_msg)
            
            # 통신 끊김 경고 로그도 0.5초 주기로 출력하도록 제한
            log_diff = (now - self.last_log_time).nanoseconds / 1e9
            if log_diff > 0.5:
                self.get_logger().warn(f'[WATCHDOG ACTIVED] No signal from follow_node for {time_diff:.2f}s! EMERGENCY STOPPED.')
                self.last_log_time = now

    def print_clean_log(self, mode, dist, angle, cmd):
        """0.5초 주기로 깔끔하게 포맷팅된 제어 로그를 출력합니다."""
        now = self.get_clock().now()
        log_diff = (now - self.last_log_time).nanoseconds / 1e9
        
        if log_diff >= 0.5:
            # 터미널에서 상태를 직관적으로 확인할 수 있도록 정리된 한 줄 로그
            log_str = (
                f"[{self.current_mode_text:^15}] "
                f"Mode: {mode:.1f} | "
                f"Target Dist: {dist:.2f}m, Ang: {angle:.2f}rad | "
                f"Output Vel: {cmd.linear.x:+.2f}m/s, Omeg: {cmd.angular.z:+.2f}rad/s | "
                f"Lidar F: {self.min_dist_front:.2f}m, B: {self.min_dist_back:.2f}m"
            )
            self.get_logger().info(log_str)
            self.last_log_time = now

    def emergency_stop(self):
        self.get_logger().warn('EMERGENCY STOP: Halting all motors...')
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emergency_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
