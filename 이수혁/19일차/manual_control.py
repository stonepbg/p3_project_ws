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
        self.last_cmd = 'stop'  # 로그 도배를 막기 위한 이전 명령 저장 변수
        self.is_moving = False
        
        # 수동 조작 기본 속도 세팅
        self.linear_speed = 0.15  # 0.15 m/s
        self.angular_speed = 0.4  # 0.4 rad/s
        
        # 와치독 감시 타이머 (주기: 0.1초)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_check)
        self.get_logger().info('🕹️ 수동 조작 제어 노드 (로그 개선 및 통신 안정화 적용) 가동')

    def send_log(self, text, level='info'):
        if level == 'info': self.get_logger().info(text)
        elif level == 'warn': self.get_logger().warn(text)
        
        # 대시보드 전시를 위해 AGV_log 토픽으로 발행
        msg = String()
        msg.data = f"🕹️ [수동조작] {text}"
        self.log_pub.publish(msg)

    def mode_callback(self, msg):
        self.current_agv_mode = msg.data

    def cmd_callback(self, msg):
        if self.current_agv_mode != 60:
            return
            
        self.last_cmd_time = time.time()
        
        # 공백 제거나 대소문자 차이로 인한 오류를 막기 위해 strip()과 lower() 적용
        cmd = msg.data.strip().lower()
        twist = Twist()
        
        if cmd == 'forward': twist.linear.x = self.linear_speed
        elif cmd == 'backward': twist.linear.x = -self.linear_speed
        elif cmd == 'left': twist.linear.y = self.linear_speed
        elif cmd == 'right': twist.linear.y = -self.linear_speed
        elif cmd == 'ccw': twist.angular.z = self.angular_speed
        elif cmd == 'cw': twist.angular.z = -self.angular_speed
        elif cmd == 'stop': pass 
            
        self.cmd_vel_pub.publish(twist)
        
        # 로그 도배 방지: 새로운 동작 명령이 들어왔을 때만 로그 전송
        if cmd != self.last_cmd and cmd != 'stop':
            self.send_log(f'명령 수신: [{cmd.upper()}] 방향 이동 시작')
            self.last_cmd = cmd
            self.is_moving = True

    def watchdog_check(self):
        if self.current_agv_mode != 60:
            return
            
        # 통신 지연(핑 튐 현상)을 고려해 0.15초에서 0.25초로 안전 마진 확보
        if self.is_moving and (time.time() - self.last_cmd_time > 0.25):
            self.cmd_vel_pub.publish(Twist())
            self.is_moving = False
            self.last_cmd = 'stop'
            self.send_log('⚠️ 명령 수신 중단. 자동 브레이크 작동.')

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
