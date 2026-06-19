import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import subprocess
import os
import signal
import time

class AGVTaskManager(Node):
    def __init__(self):
        super().__init__('agv_task_manager')
        
        # 1. 외부 FSM이 보내는 모드 구독 (1회성 신호 캐치용)
        self.mode_sub = self.create_subscription(Int32, '/agv_mode', self.mode_callback, 10)
        
        # 2. AGV 내부 노드들에게 현재 모드를 지속적으로 알려줄 퍼블리셔 추가
        self.internal_mode_pub = self.create_publisher(Int32, '/internal_mode', 10)
        
        self.current_mode = 0
        self.active_process = None
        
        # 0.5초마다 현재 모드를 내부망에 방송하는 타이머
        self.broadcast_timer = self.create_timer(0.5, self.broadcast_current_mode)
        
        self.get_logger().info('🛠️ AGV 태스크 매니저 가동 완료: 외부 FSM의 명령을 대기합니다.')

    def broadcast_current_mode(self):
        # 켜져 있는 하위 노드들이 언제든 현재 상태를 알 수 있도록 0.5초마다 계속 전송
        msg = Int32()
        msg.data = self.current_mode
        self.internal_mode_pub.publish(msg)

    def mode_callback(self, msg):
        new_mode = msg.data
        
        new_group = self.get_mode_group(new_mode)
        old_group = self.get_mode_group(self.current_mode)
        
        self.current_mode = new_mode

        if new_group != old_group:
            self.stop_current_process()
            
            if new_group == 1:
                self.start_process(['ros2', 'launch', 'mode1_follow.launch.py'])
            elif new_group == 2:
                self.start_process(['ros2', 'launch', 'mode2_guide.launch.py'])
            elif new_group == 3:
                self.start_process(['ros2', 'launch', 'mode3_goto.launch.py'])
            elif new_group == 0:
                self.get_logger().info('⏹️ 대기 모드: 모든 활성 노드를 종료하고 대기 상태를 유지합니다.')

    def get_mode_group(self, mode):
        if mode == 10: 
            return 1
        elif 20 <= mode <= 27: 
            return 2
        elif 30 <= mode <= 37: 
            return 3
        else: 
            return 0

    def start_process(self, cmd_list):
        self.get_logger().info(f'🚀 새로운 작업 시작: {" ".join(cmd_list)}')
        self.active_process = subprocess.Popen(cmd_list, preexec_fn=os.setsid)

    def stop_current_process(self):
        if self.active_process is not None:
            self.get_logger().info('🛑 기존 작업 런치 파일을 종료합니다...')
            try:
                os.killpg(os.getpgid(self.active_process.pid), signal.SIGINT)
                self.active_process.wait(timeout=5.0) 
                self.get_logger().info('✅ 기존 작업 종료 완료.')
            except Exception as e:
                self.get_logger().warn(f'프로세스 종료 중 예외 발생: {e}')
            finally:
                self.active_process = None
                time.sleep(1.0) 

def main(args=None):
    rclpy.init(args=args)
    node = AGVTaskManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('태스크 매니저 종료 감지. 실행 중인 프로세스를 모두 정리합니다.')
        node.stop_current_process()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()