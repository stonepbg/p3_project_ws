#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Empty, Bool
from geometry_msgs.msg import PoseStamped
# ──────────────────────────────────────
# TODO: 아래 두 자세의 실제 좌표를 실측 후 교체
#   POSE_1: 카메라가 정면을 향하는 자세
#   POSE_2: 카메라가 후면을 향하는 자세
# 단위: position [m], orientation은 quat_xyzw, frame_id는 g_base 기준
# ──────────────────────────────────────
POSE_1 = {
    "position": (-0.1056, -0.0625, 0.3546),
    "quat_xyzw": (-0.5757, 0.3011, -0.7103, 0.2708),
}
POSE_2 = {
    "position": (0.0342, 0.0547, 0.3668),
    "quat_xyzw": (0.2782, 0.6257, -0.2816, -0.6722),
}

POSE_3 = {
    "position": (-0.0848, -0.0658, 0.2506),
    "quat_xyzw": (0.6655, -0.3487, 0.6218, -0.2212)
}
class PosePresetController(Node):
    def __init__(self):
        super().__init__('pose_preset_controller')
        self.pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        # 마지막으로 발행한 모드 (1 또는 2). 시작 시 None이라
        # ready 신호를 받은 뒤 첫 발행이 무조건 실행됨.
        self.last_mode = None
        self._initial_pose_sent = False
        self._awaiting_pose3_done = False
        self.agv_mode_sub = self.create_subscription(
            Int32,
            '/agv_mode',
            self.agv_mode_callback,
            10,
        )
        self.get_logger().info("'/agv_mode' 토픽 구독 시작")
        # pose_target_controller가 준비됐다는 신호를 받을 때까지 대기
        self.ready_sub = self.create_subscription(
            Empty,
            '/pose_target_ready',
            self._on_pose_target_ready,
            10,
        )
        self.get_logger().info("'/pose_target_ready' 신호 대기 중")
        self.done_sub = self.create_subscription(
            Bool,
            '/pose_target_done',
            self._on_pose_target_done,
            10,
        )
    def _on_pose_target_ready(self, msg: Empty):
        if self._initial_pose_sent:
            return
        self._initial_pose_sent = True
        self.get_logger().info("pose_target_controller 준비 확인됨")
        self.publish_preset(1, POSE_1)
    def _on_pose_target_done(self, msg: Bool):
        if not self._awaiting_pose3_done:
            return
        self._awaiting_pose3_done = False
        if not msg.data:
            self.get_logger().error("POSE_3(준비자세) 이동 실패 — 정지, 재시도 없음")
    def agv_mode_callback(self, msg: Int32):
        value = msg.data
        if value == 10 or 30 <= value <= 39:
            target_mode = 1
        elif 20 <= value <= 29:
            target_mode = 2
        elif value in (50, 51):
            target_mode = 3
        else:
            self.get_logger().info(f"agv_mode={value} 무시 (처리 대상 아님)")
            return
        if target_mode == 1:
            pose = POSE_1
        elif target_mode == 2:
            pose = POSE_2
        else:
            pose = POSE_3
        self.publish_preset(target_mode, pose)
    def publish_preset(self, mode: int, pose: dict):
        msg = PoseStamped()
        msg.header.frame_id = 'g_base'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = pose["position"][0]
        msg.pose.position.y = pose["position"][1]
        msg.pose.position.z = pose["position"][2]
        msg.pose.orientation.x = pose["quat_xyzw"][0]
        msg.pose.orientation.y = pose["quat_xyzw"][1]
        msg.pose.orientation.z = pose["quat_xyzw"][2]
        msg.pose.orientation.w = pose["quat_xyzw"][3]
        self.pub.publish(msg)
        self.get_logger().info(f"{mode}번 자세 좌표를 /target_pose로 발행함")
        self.last_mode = mode
        if mode == 3:
            self._awaiting_pose3_done = True
def main(args=None):
    rclpy.init(args=args)
    node = PosePresetController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
