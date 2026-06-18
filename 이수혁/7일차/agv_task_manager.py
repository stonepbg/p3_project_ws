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
        
        # 외부 메인 FSM이 보내는 모드 구독
        self.mode_sub = self.create_subscription(Int32, '/agv_mode', self.mode_callback, 10)
        
        self.current_mode = 0
        self.active_process = None
        
        self.get_logger().info('🛠️ AGV 태스크 매니저 가동 완료: 외부 FSM의 명령을 대기합니다.')

    def mode_callback(self, msg):
        new_mode = msg.data
        
        # 모드 번호가 속한 "그룹"을 파악 (0: 정지, 1: 추종, 2: 안내, 3: 단순이동)
        new_group = self.get_mode_group(new_mode)
        old_group = self.get_mode_group(self.current_mode)
        
        self.current_mode = new_mode

        # 모드 그룹이 변경되었을 때만 런치 파일을 새로 켜거나 끔
        # (예: 21번 가다가 23번으로 바뀐 경우는 그룹이 같으므로 런치 파일을 껐다 켤 필요 없음)
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
        # 프로세스 그룹으로 묶어서 실행 (나중에 하위 노드들까지 한 번에 깔끔하게 끄기 위함)
        self.active_process = subprocess.Popen(cmd_list, preexec_fn=os.setsid)

    def stop_current_process(self):
        if self.active_process is not None:
            self.get_logger().info('🛑 기존 작업 런치 파일을 종료합니다...')
            try:
                # 프로세스 그룹 전체에 Ctrl+C (SIGINT) 신호 전송
                os.killpg(os.getpgid(self.active_process.pid), signal.SIGINT)
                self.active_process.wait(timeout=5.0) # 안전하게 꺼질 때까지 최대 5초 대기
                self.get_logger().info('✅ 기존 작업 종료 완료.')
            except Exception as e:
                self.get_logger().warn(f'프로세스 종료 중 예외 발생 (강제 종료됨): {e}')
            finally:
                self.active_process = None
                time.sleep(1.0) # 포트 충돌 방지를 위해 잠시 대기

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