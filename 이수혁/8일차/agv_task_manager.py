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
        
        self.mode_sub = self.create_subscription(Int32, '/agv_mode', self.mode_callback, 10)
        self.ack_pub = self.create_publisher(Int32, '/AGV_mode_ack', 10)
        self.internal_mode_pub = self.create_publisher(Int32, '/internal_mode', 10)
        
        self.current_mode = 0
        self.active_process = None
        self.broadcast_timer = self.create_timer(0.5, self.broadcast_current_mode)
        
        self.get_logger().info('🛠️ AGV 태스크 매니저 가동: FSM 명령 대기 및 Ack 시스템 활성화')

    def broadcast_current_mode(self):
        msg = Int32()
        msg.data = self.current_mode
        self.internal_mode_pub.publish(msg)

    def mode_callback(self, msg):
        new_mode = msg.data
        
        # 🚨 [수신 증거 로그] 이 로그가 안 뜬다면 FSM이 안 쏜 것이거나 통신 단절입니다!
        self.get_logger().info(f'📥 [수신됨] 외부 FSM으로부터 [{new_mode}]번 명령이 도착했습니다.')
        
        # 받자마자 무조건 똑같은 번호로 Ack 회신 (FSM이 송신을 멈추게 함)
        ack_msg = Int32()
        ack_msg.data = new_mode
        self.ack_pub.publish(ack_msg)
        
        new_group = self.get_mode_group(new_mode)
        old_group = self.get_mode_group(self.current_mode)
        self.current_mode = new_mode
        
        # 모드가 바뀌면 즉시 내부망에 방송하여 하위 노드들이 브레이크를 밟게 함
        self.broadcast_current_mode()

        if new_group != old_group:
            self.stop_current_process()
            
            if new_group == 1:
                self.start_process(['ros2', 'launch', 'mode1_follow.launch.py'])
            elif new_group == 2:
                self.start_process(['ros2', 'launch', 'mode2_guide.launch.py'])
            elif new_group == 3:
                self.start_process(['ros2', 'launch', 'mode3_goto.launch.py'])
            elif new_group == 0:
                self.get_logger().info('⏹️ 대기 모드: 로봇을 완전히 정지하고 대기합니다.')

    def get_mode_group(self, mode):
        if mode == 10: return 1
        elif 20 <= mode <= 27: return 2
        elif 30 <= mode <= 37: return 3
        else: return 0

    def start_process(self, cmd_list):
        self.get_logger().info(f'🚀 프로세스 실행: {" ".join(cmd_list)}')
        self.active_process = subprocess.Popen(cmd_list, preexec_fn=os.setsid)

    def stop_current_process(self):
        if self.active_process is not None:
            self.get_logger().info('🛑 모터 정지 신호 전달을 위해 0.5초 대기...')
            time.sleep(0.5) 
            
            self.get_logger().info('🛑 기존 프로세스를 안전하게 종료합니다.')
            try:
                os.killpg(os.getpgid(self.active_process.pid), signal.SIGINT)
                self.active_process.wait(timeout=5.0) 
                self.get_logger().info('✅ 종료 완료.')
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
        node.stop_current_process()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()