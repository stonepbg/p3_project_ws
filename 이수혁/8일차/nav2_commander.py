import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import math
import time

class Nav2Commander(Node):
    def __init__(self):
        super().__init__('nav2_commander')

        self.mode_sub = self.create_subscription(Int32, '/internal_mode', self.mode_callback, 10)
        self.pause_sub = self.create_subscription(Bool, '/guide_pause', self.pause_callback, 10)
        
        self.status_pub = self.create_publisher(Int32, '/AGV_status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.marker_database = {
            0: (0.055, -0.382, 3.12),
            1: (1.759, -0.202, 1.57),
            2: (1.457, -2.864, 0.03),
            3: (1.567, -4.364, 0.01),
            4: (1.561, -5.816, 0.00),
            5: (1.440, -7.275, 0.00),
            6: (1.485, -8.477, -1.57),
            7: (0.142, -8.044, -3.13)
        }

        self.current_mode = 0
        self.current_target_id = None
        self.goal_handle = None
        self.is_paused = False
        
        self.get_logger().info('🗺️ Nav2 자율주행 커맨더 가동 완료. 명령 대기 중...')

    def mode_callback(self, msg):
        mode = msg.data
        if mode == self.current_mode:
            return 
            
        self.current_mode = mode

        if mode == 0 or mode == 10:
            self.cancel_current_goal()
            return

        if 20 <= mode <= 27:
            self.current_target_id = mode - 20
            self.is_paused = False
            self.get_logger().info(f'💁 [안내 모드] {self.current_target_id}번 목적지로 안내를 시작합니다.')
            self.send_nav_goal(self.current_target_id)

        elif 30 <= mode <= 37:
            self.current_target_id = mode - 30
            self.is_paused = False
            self.get_logger().info(f'🚚 [이동 모드] {self.current_target_id}번 목적지로 이동을 시작합니다.')
            self.send_nav_goal(self.current_target_id)

    def pause_callback(self, msg):
        if 20 <= self.current_mode <= 27:
            should_pause = msg.data
            if should_pause and not self.is_paused:
                self.get_logger().warn('⚠️ 사람 놓침! 안내 주행 일시 정지.')
                self.is_paused = True
                self.cancel_current_goal() 
            elif not should_pause and self.is_paused:
                self.get_logger().info('▶️ 사람 재인식! 안내 주행 재개.')
                self.is_paused = False
                self.send_nav_goal(self.current_target_id) 

    def send_nav_goal(self, target_id):
        if target_id not in self.marker_database:
            self.get_logger().error(f'❌ 알 수 없는 목적지 ID입니다: {target_id}')
            return

        x, y, yaw = self.marker_database[target_id]
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_current_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info('🛑 진행 중인 주행 취소.')
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.cmd_vel_pub.publish(Twist())

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('❌ Nav2에서 주행 경로 생성이 거부되었습니다.')
            return
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4: # SUCCEEDED
            self.get_logger().info('🏁 목적지에 안전하게 도착했습니다.')
            
            if 30 <= self.current_mode <= 37:
                self.perform_180_turn()
            
            # [핵심] 죽기 전에 FSM과 매니저가 확실히 듣도록 1 신호를 빠르게 3연발 쏩니다.
            msg = Int32()
            msg.data = 1
            for _ in range(3):
                self.status_pub.publish(msg)
                time.sleep(0.1)
                
            self.get_logger().info('📡 도착 완료(1) 신호를 전송했습니다. 매니저의 종료 명령을 대기합니다.')

    def perform_180_turn(self):
        self.get_logger().info('🔄 [모드 3] 제자리 180도 회전을 수행합니다.')
        twist = Twist()
        twist.angular.z = 0.5 
        turn_duration = math.pi / 0.5
        start_time = time.time()

        while time.time() - start_time < turn_duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('✅ 180도 회전 완료.')

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Commander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_vel_pub.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()