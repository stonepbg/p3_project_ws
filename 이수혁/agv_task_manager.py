import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import subprocess
import os
import signal
import time

class AGVTaskManager(Node):
    def __init__(self):
        super().__init__('agv_task_manager')
        
        self.mode_sub = self.create_subscription(Int32, '/agv_mode', self.mode_callback, 10)
        self.status_sub = self.create_subscription(Int32, '/AGV_status', self.status_callback, 10)
        
        self.ack_pub = self.create_publisher(Int32, '/AGV_mode_ack', 10)
        self.internal_mode_pub = self.create_publisher(Int32, '/internal_mode', 10)
        self.log_pub = self.create_publisher(String, '/AGV_log', 10)
        
        self.current_mode = 0
        self.active_process = None
        self.broadcast_timer = self.create_timer(0.5, self.broadcast_current_mode)
        
        self.get_logger().info('🛠️ AGV 태스크 매니저 가동 (50번/51번 픽업 모드 라우팅 추가)')

    def send_log(self, text, level='info'):
        if level == 'info': self.get_logger().info(text)
        elif level == 'warn': self.get_logger().warn(text)
        
        msg = String()
        msg.data = f"[매니저] {text}"
        self.log_pub.publish(msg)

    def broadcast_current_mode(self):
        msg = Int32()
        msg.data = self.current_mode
        self.internal_mode_pub.publish(msg)

    def status_callback(self, msg):
        if msg.data == 1 and self.current_mode != 0:
            self.send_log('🏁 [자율 종료] 목적지 도착(1) 감지. 대기 모드로 전환합니다.')
            self.current_mode = 0
            self.broadcast_current_mode()
            self.stop_current_process()
            
        elif msg.data == 0 and self.current_mode == 40:
            self.send_log('🎯 [자율 종료] 위치 캘리브레이션(0) 완료. 대기 모드로 전환합니다.')
            self.current_mode = 0
            self.broadcast_current_mode()
            self.stop_current_process()

    def mode_callback(self, msg):
        new_mode = msg.data
        self.send_log(f'📥 [수신됨] 외부로부터 [{new_mode}]번 명령 도착')
        
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
                # [수정됨] 10번(일반 추종)과 50번(픽업 유도)은 같은 코드를 씁니다.
                self.start_process(['ros2', 'launch', 'mode1_follow.launch.py'])
            elif new_group == 2:
                self.start_process(['ros2', 'launch', 'mode_navigate.launch.py'])
            elif new_group == 4:
                self.start_process(['ros2', 'launch', 'vision_localization.launch.py'])
            elif new_group == 5:
                # [신규] 51번 픽업 정밀 밀착 전용 런치 (다음 단계에서 생성)
                self.start_process(['ros2', 'launch', 'mode5_pickup.launch.py'])
            elif new_group == 0:
                self.send_log('⏹️ 대기 모드: 모든 동작을 강제 정지합니다.')

    def get_mode_group(self, mode):
        if mode in [10, 50]: return 1  # 50번 추가
        elif 20 <= mode <= 37: return 2  
        elif mode == 40: return 4
        elif mode == 51: return 5      # 51번 신규 추가
        else: return 0

    def start_process(self, cmd_list):
        self.send_log(f'🚀 런치 파일 실행: {cmd_list[-1]}')
        self.active_process = subprocess.Popen(cmd_list, preexec_fn=os.setsid)

    def stop_current_process(self):
        if self.active_process is not None:
            self.send_log('🛑 모드 전환을 위해 기존 프로세스를 즉시 종료합니다.')
            try:
                os.killpg(os.getpgid(self.active_process.pid), signal.SIGINT)
                self.active_process.wait(timeout=3.0) 
            except Exception as e:
                self.get_logger().warn(f'종료 예외: {e}')
            finally:
                self.active_process = None
                time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = AGVTaskManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.stop_current_process()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()