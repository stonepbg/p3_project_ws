import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
import time

class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')
        
        self.mode_sub = self.create_subscription(Int32, '/internal_mode', self.mode_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/manual_cmd', self.cmd_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.log_pub = self.create_publisher(String, '/AGV_log', 10)
        
        self.current_agv_mode = 0
        self.last_cmd_time = 0.0
        self.is_moving = False
        
        # 수동 조작 기본 속도 세팅
        self.linear_speed = 0.15  # 0.15 m/s (안전 속도)
        self.angular_speed = 0.4  # 0.4 rad/s
        
        # 0.05초 주기로 통신 유실을 감시하는 와치독 타이머
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_check)
        self.get_logger().info('🕹️ 수동 조작 제어 노드(안전 와치독 포함) 가동')

    def send_log(self, text):
        msg = String()
        msg.data = f"[수동제어] {text}"
        self.log_pub.publish(msg)

    def mode_callback(self, msg):
        self.current_agv_mode = msg.data

    def cmd_callback(self, msg):
        if self.current_agv_mode != 60:
            return
            
        self.last_cmd_time = time.time()
        cmd = msg.data.lower()
        twist = Twist()
        
        if cmd == 'forward':
            twist.linear.x = self.linear_speed
        elif cmd == 'backward':
            twist.linear.x = -self.linear_speed
        elif cmd == 'left':
            twist.linear.y = self.linear_speed
        elif cmd == 'right':
            twist.linear.y = -self.linear_speed
        elif cmd == 'ccw':
            twist.angular.z = self.angular_speed
        elif cmd == 'cw':
            twist.angular.z = -self.angular_speed
        elif cmd == 'stop':
            pass # 모두 0인 상태 유지
            
        self.cmd_vel_pub.publish(twist)
        
        if not self.is_moving and cmd != 'stop':
            self.send_log(f'동작 시작: {cmd}')
            self.is_moving = True

    def watchdog_check(self):
        if self.current_agv_mode != 60:
            return
            
        # 마지막 명령이 들어온 지 0.15초가 지나면 버튼에서 손을 뗀 것으로 간주하고 정지
        if self.is_moving and (time.time() - self.last_cmd_time > 0.15):
            self.cmd_vel_pub.publish(Twist())
            self.is_moving = False
            self.send_log('명령 수신 없음. 자동 정지합니다.')

def main(args=None):
    rclpy.init(args=args)
    node = ManualControl()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()