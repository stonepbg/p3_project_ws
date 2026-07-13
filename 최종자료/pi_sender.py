#!/usr/bin/env python3
import socket
import json
import threading
import rclpy
from rclpy.node import Node
from mycobot_interfaces.msg import MycobotSetAngles
# =====================================================================
# ⚠️ 기존 CycloneDDS 관련 os.environ 하드코딩 라인들을 모두 제거했습니다.
# (이제 launch_arm.sh에서 설정한 FastDDS 환경 변수를 그대로 이어받습니다)
# =====================================================================
class PiSenderNode(Node):
    def __init__(self):
        super().__init__('pi_sender')
        self.pub = self.create_publisher(MycobotSetAngles, '/mycobot/angles_goal', 10)
        # 로그 메시지를 FastDDS에 맞게 수정했습니다.
        self.get_logger().info('[Pi 송신] 시작 (FastDDS, 독립 프로세스)')
    # myCobot 280 관절별 허용 범위
    JOINT_LIMITS = [
        (-165.0, 165.0),
        (-135.0, 135.0),
        (-165.0, 165.0),
        (-145.0, 145.0),
        (-165.0, 165.0),
        (-175.0, 175.0),
    ]
    def send_angles(self, angles, speed):
        clamped = [
            max(lo, min(hi, float(a)))
            for a, (lo, hi) in zip(angles, self.JOINT_LIMITS)
        ]
        if clamped != [float(a) for a in angles]:
            self.get_logger().warn(
                f"각도 범위 초과로 클램핑: {[round(a,1) for a in angles]}"
                f" → {[round(a,1) for a in clamped]}"
            )
        msg = MycobotSetAngles()
        msg.joint_1 = clamped[0]
        msg.joint_2 = clamped[1]
        msg.joint_3 = clamped[2]
        msg.joint_4 = clamped[3]
        msg.joint_5 = clamped[4]
        msg.joint_6 = clamped[5]
        msg.speed = int(speed)
        self.pub.publish(msg)
def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 9999))
    sock.settimeout(1.0)
    rclpy.init()
    node = PiSenderNode()
    def recv_loop():
        while rclpy.ok():
            try:
                data, _ = sock.recvfrom(4096)
                payload = json.loads(data.decode())
                node.send_angles(payload['angles'], payload['speed'])
            except socket.timeout:
                continue
            except Exception as e:
                node.get_logger().error(f'오류: {e}')
    t = threading.Thread(target=recv_loop, daemon=True)
    t.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
