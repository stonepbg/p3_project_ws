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
        
        # [핵심 추가] 하위 노드가 임무를 끝냈는지 감시하기 위해 상태 토픽 구독
        self.status_sub = self.create_subscription(Int32, '/AGV_status', self.status_callback, 10)
        
        self.ack_pub = self.create_publisher(Int32, '/AGV_mode_ack', 10)
        self.internal_mode_pub = self.create_publisher(Int32, '/internal_mode', 10)
        
        self.current_mode = 0
        self.active_process = None
        self.broadcast_timer = self.create_timer(0.5, self.broadcast_current_mode)
        
        self.get_logger().info('🛠️ AGV 태스크 매니저 가동: FSM 명령 대기 및 자율 종료 시스템 활성화')

    def broadcast_current_mode(self):
        msg = Int32()
        msg.data = self.current_mode
        self.internal_mode_pub.publish(msg)

    def status_callback(self, msg):
        # [자율 종료 로직] 커맨더가 "도착(1)" 신호를 쏘면 매니저가 스스로 시스템을 종료합니다.
        if msg.data == 1 and self.current_mode != 0:
            self.get_logger().info('🏁 [자율 종료] 목적지 도착(1)을 감지했습니다. FSM의 0번 명령 없이 스스로 대기 모드(0)로 전환합니다.')
            self.current_mode = 0
            self.broadcast_current_mode() # 0번을 방송해서 하위 노드 브레이크 작동
            self.stop_current_process()   # 프로세스 종료

    def mode_callback(self, msg):
        new_mode = msg.data
        
        # FSM 통신망 확인 로그 및 Ack 회신
        self.get_logger().info(f'📥 [수신됨] 외부 FSM으로부터 [{new_mode}]번 명령 도착')
        ack_msg = Int32()
        ack_msg.data = new_mode
        self.ack_pub.publish(ack_msg)
        
        new_group = self.get_mode_group(new_mode)
        old_group = self.get_mode_group(self.current_mode)
        self.current_mode = new_mode
        
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
                self.get_logger().info('⏹️ 대기 모드: 강제 정지 명령을 수행합니다.')

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
            
            self.get_logger().info('🛑 런치 파일을 안전하게 종료합니다.')
            try:
                os.killpg(os.getpgid(self.active_process.pid), signal.SIGINT)
                self.active_process.wait(timeout=5.0) 
                self.get_logger().info('✅ 프로세스 종료 완료.')
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