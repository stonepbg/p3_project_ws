#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Bool, Empty
from mycobot_interfaces.msg import MycobotGripperStatus

import tf2_ros
import tf2_geometry_msgs  # noqa: F401


# 집기 orientation (수평 정면 고정)
GRASP_QUAT = (0.6291, -0.2825, 0.6654, -0.2859)

# GRASP_QUAT 자세(수평 정면 고정)에서 플랜지-그리퍼 사이 x축 거리 (실측값, m).
GRASP_TCP_OFFSET_X = 0.07
GRASP_Y_OFFSET = 0.06

# 접근/집기 높이 오프셋 (m)
APPROACH_HEIGHT_OFFSET = 0.023


POSE_READY = {
    "position": (-0.0848, -0.0658, 0.2506),
    "quat_xyzw": (0.6655, -0.3487, 0.6218, -0.2212)
}
POSE_DROP = {
         "position": (-0.2778, 0.0105, 0.1551),
         "quat_xyzw": (0.3820, 0.9238, 0.0217, -0.0137),
}

GRIPPER_GRIP_THRESHOLD = 30  # 이 값 미만이면 파지 실패


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')

        callback_group = ReentrantCallbackGroup()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.gripper_pub = self.create_publisher(
            MycobotGripperStatus, '/mycobot/gripper_status', 10
        )
        self.target_pickup_pub = self.create_publisher(
            PoseStamped, '/target_pose_pickup', 10
        )
        self.target_pub = self.create_publisher(
            PoseStamped, '/target_pose', 10
        )
        self.scene_object_pub = self.create_publisher(
            PoseStamped, '/scene_object_pose', 10
        )
        self.pedestal_remove_pub = self.create_publisher(
            Empty, '/pedestal_remove', 10
        )
        self.arm_status_pub = self.create_publisher(
            Int32, '/ARM_status', 10
        )

        self._done = False
        self._done_success = False
        self.done_sub = self.create_subscription(
            Bool,
            '/pose_target_done',
            self._on_done,
            10,
            callback_group=callback_group,
        )

        self._gripper_value = None
        self.create_subscription(
            Int32,
            '/gripper_value',
            self._on_gripper_value,
            10,
            callback_group=callback_group,
        )

        self._busy = False
        self._pick_mode = False

        self.create_subscription(
            Int32,
            '/agv_mode',
            self._agv_mode_callback,
            10,
            callback_group=callback_group,
        )

        self.sub = self.create_subscription(
            PoseStamped,
            '/object_pose',
            self.object_pose_callback,
            10,
            callback_group=callback_group,
        )

        self.get_logger().info("pick_place_node 시작. /object_pose 대기 중")

    # ──────────────────────────────────
    # 유틸리티
    # ──────────────────────────────────

    def _on_done(self, msg: Bool):
        self._done = True
        self._done_success = msg.data

    def _on_gripper_value(self, msg: Int32):
        self._gripper_value = msg.data

    def move(self, position, quat_xyzw, label="", timeout=15.0, tight=False):
        """/target_pose_pickup을 발행하고 완료 신호를 기다림."""
        msg = PoseStamped()
        msg.header.frame_id = 'g_base'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.x = float(quat_xyzw[0])
        msg.pose.orientation.y = float(quat_xyzw[1])
        msg.pose.orientation.z = float(quat_xyzw[2])
        msg.pose.orientation.w = float(quat_xyzw[3])

        self._done = False
        self._done_success = False
        pub = self.target_pub if tight else self.target_pickup_pub
        pub.publish(msg)
        self.get_logger().info(f"[{label}] /target_pose_pickup 발행")

        start = time.time()
        while not self._done:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().error(f"[{label}] 완료 신호 타임아웃")
                return False

        if self._done_success:
            self.get_logger().info(f"[{label}] 완료")
        else:
            self.get_logger().warn(f"[{label}] 실패")

        return self._done_success

    def _return_to_ready(self, label="준비자세 복귀"):
        self.pedestal_remove_pub.publish(Empty())
        time.sleep(0.3)
        if not self.move(POSE_READY["position"], POSE_READY["quat_xyzw"], label=label, tight=True):
            self.get_logger().error(f"[{label}] 실패 — 정지 (재시도 없음)")
            return False
        return True

    def _wait_gripper_value(self, timeout=3.0):
        """gripper_close() 후 /gripper_value 수신 대기. 반환값: int or None"""
        self._gripper_value = None
        start = time.time()
        while self._gripper_value is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().warn("gripper_value 수신 타임아웃")
                return None
        return self._gripper_value

    def gripper_open(self):
        msg = MycobotGripperStatus()
        msg.status = True
        self.gripper_pub.publish(msg)
        self.get_logger().info("그리퍼 open")
        time.sleep(0.5)

    def gripper_close(self):
        msg = MycobotGripperStatus()
        msg.status = False
        self.gripper_pub.publish(msg)
        self.get_logger().info("그리퍼 close")
        time.sleep(0.5)

    # ──────────────────────────────────
    # 콜백
    # ──────────────────────────────────

    def _agv_mode_callback(self, msg: Int32):
        if msg.data in (50, 51):
            if not self._pick_mode:
                self._pick_mode = True
                self.get_logger().info("pick 모드 활성화 (agv_mode=50/51)")
        else:
            if self._pick_mode:
                self._pick_mode = False
                self.get_logger().info(
                    f"pick 모드 비활성화 (agv_mode={msg.data})"
                )

    def object_pose_callback(self, msg: PoseStamped):
        if not self._pick_mode:
            return
        if self._busy:
            self.get_logger().warn("작업 중 — 새 좌표 무시")
            return

        self._busy = True
        try:
            self._pick_and_place(msg)
        except Exception as e:
            self.get_logger().error(f"pick-and-place 오류: {e}")
            self._busy = False

    def _pick_and_place(self, msg: PoseStamped):
        # 1. TF 변환: camera_link → g_base
        if msg.header.frame_id == 'camera_link':
            msg.header.frame_id = 'gripper_camera_link'

        if msg.header.frame_id != 'g_base':
            try:
                msg.header.stamp = rclpy.time.Time().to_msg()
                msg = self.tf_buffer.transform(
                    msg, 'g_base',
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
            except Exception as e:
                self.get_logger().error(f"TF 변환 실패: {e}")
                self._return_to_ready("준비자세 복귀(TF 실패)")
                self._busy = False
                return

        ox = msg.pose.position.x
        oy = msg.pose.position.y
        oz = msg.pose.position.z
        self.get_logger().info(
            f"물건 좌표 (g_base): x={ox:.3f}, y={oy:.3f}, z={oz:.3f}"
        )

        # 2. 그리퍼 open
        self.gripper_open()

        # 2-1. 집기 전 경유
        if not self.move(
            (ox - GRASP_TCP_OFFSET_X - 0.04, oy + GRASP_Y_OFFSET, oz + APPROACH_HEIGHT_OFFSET + 0.03),
            GRASP_QUAT, label="경유(집기 전)", tight=True
        ):
            self._return_to_ready("준비자세 복귀(경유(집기 전) 실패)")
            self._busy = False
            time.sleep(2.0)
            arm_msg = Int32()
            arm_msg.data = 0
            self.arm_status_pub.publish(arm_msg)
            return

        time.sleep(2.0)
        # 3. 물건 위치로 이동 (집기)
        if not self.move(
            (ox - GRASP_TCP_OFFSET_X, oy + GRASP_Y_OFFSET, oz + APPROACH_HEIGHT_OFFSET),
            GRASP_QUAT, label="집기", tight=True
        ):
            self._return_to_ready("준비자세 복귀(집기 실패)")
            self._busy = False
            time.sleep(2.0)
            arm_msg = Int32()
            arm_msg.data = 0
            self.arm_status_pub.publish(arm_msg)
            return
        time.sleep(7.0)

        # 4. 그리퍼 close
        self.gripper_close()

        # 4-1. 파지 성공 여부 확인 (Pi에서 gripper_value 수신)
        gval = self._wait_gripper_value(timeout=3.0)
        if gval is None or gval < GRIPPER_GRIP_THRESHOLD:
            self.get_logger().warn(f"파지 실패 (gripper_value={gval} < {GRIPPER_GRIP_THRESHOLD})")
            self.gripper_open()
            self.move(
                (ox - GRASP_TCP_OFFSET_X - 0.03, oy + GRASP_Y_OFFSET, oz + APPROACH_HEIGHT_OFFSET + 0.05),
                GRASP_QUAT, label="경유(위)(파지 실패)"
            )
            self._return_to_ready("준비자세 복귀(파지 실패)")
            self._busy = False
            time.sleep(2.0)
            arm_msg = Int32()
            arm_msg.data = 0
            self.arm_status_pub.publish(arm_msg)
            return
        self.get_logger().info(f"파지 성공 (gripper_value={gval})")
        time.sleep(2.0)

        # 가판대 충돌 객체 등록 (물건 자체는 충돌 객체로 등록 안 함)
        scene_msg = PoseStamped()
        scene_msg.header.frame_id = 'g_base'
        scene_msg.header.stamp = self.get_clock().now().to_msg()
        scene_msg.pose.position.x = ox
        scene_msg.pose.position.y = oy
        scene_msg.pose.position.z = oz
        scene_msg.pose.orientation.w = 1.0
        self.scene_object_pub.publish(scene_msg)
        time.sleep(0.5)

        # 4-2. 위로 경유
        if not self.move(
            (ox - GRASP_TCP_OFFSET_X - 0.03, oy + GRASP_Y_OFFSET, oz + APPROACH_HEIGHT_OFFSET + 0.05),
            GRASP_QUAT, label="경유(위)"
        ):
            self.gripper_open()
            self._return_to_ready("준비자세 복귀(경유 실패)")
            self._busy = False
            time.sleep(2.0)
            arm_msg = Int32()
            arm_msg.data = 0
            self.arm_status_pub.publish(arm_msg)
            return
        time.sleep(2.0)

        # 5. 놓는 위치로 이동
        if not self.move(POSE_DROP["position"], POSE_DROP["quat_xyzw"], label="이동(놓기)"):
            self.gripper_open()
            self._return_to_ready("준비자세 복귀(이동(놓기) 실패)")
            self._busy = False
            return
        time.sleep(2.0)

        # 6. 4초 대기 후 그리퍼 open
        time.sleep(4.0)
        self.gripper_open()
        time.sleep(2.0)

        # 7. busy 해제 + ARM_status 1 발행
        self._busy = False
        arm_msg = Int32()
        arm_msg.data = 1
        self.arm_status_pub.publish(arm_msg)
        self.get_logger().info("pick-and-place 완료 — ARM_status 1 발행")

        # 8. 준비 자세로 복귀 (busy 밖에서 실행)
        self._return_to_ready("준비자세 복귀")


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
