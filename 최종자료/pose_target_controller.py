#!/usr/bin/env python3
"""MoveIt2 목표 자세를 수신하여 plan/execute 후 Pi로 관절 각도를 전송하는 노드."""
import os
import math
import time
import socket
import json
import subprocess
import sys

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Bool

from pymoveit2 import MoveIt2


PEDESTAL_SIZE = (0.30, 0.10, 0.20)  # 가로(접근방향과 수직), 세로(접근방향과 나란함), 두께
OBJECT_SIZE = (0.03, 0.03, 0.03)  # 충돌 객체로 등록 안 함, 가판대 위치 계산용 치수로만 사용


class PoseTargetController(Node):
    """PoseStamped 목표를 구독하여 MoveIt2로 실행하고 관절 각도를 Pi에 전송하는 ROS2 노드."""

    def __init__(self):
        """노드 초기화: MoveIt2, 소켓, 구독/발행자 설정."""
        super().__init__('pose_target_controller')

        callback_group = ReentrantCallbackGroup()

        self.joint_names = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.joint_names,
            base_link_name="g_base",
            end_effector_name="joint6_flange",
            group_name="arm_group",
            callback_group=callback_group,
        )

        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # noqa: R1732

        self.get_logger().info("pymoveit2 초기화 완료")

        self.sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.target_pose_callback,
            10,
            callback_group=callback_group,
        )
        self.get_logger().info("'/target_pose' 토픽 구독 시작")

        self.sub_pickup = self.create_subscription(
            PoseStamped,
            '/target_pose_pickup',
            self.target_pose_pickup_callback,
            10,
            callback_group=callback_group,
        )
        self.get_logger().info("'/target_pose_pickup' 토픽 구독 시작")

        self.sub_cartesian = self.create_subscription(
            PoseStamped,
            '/target_pose_cartesian',
            self.target_pose_cartesian_callback,
            10,
            callback_group=callback_group,
        )
        self.get_logger().info("'/target_pose_cartesian' 토픽 구독 시작")

        self.sub_scene_object = self.create_subscription(
            PoseStamped,
            '/scene_object_pose',
            self.scene_object_pose_callback,
            10,
            callback_group=callback_group,
        )
        self.get_logger().info("'/scene_object_pose' 토픽 구독 시작")

        self.sub_pedestal_remove = self.create_subscription(
            Empty,
            '/pedestal_remove',
            self.pedestal_remove_callback,
            10,
            callback_group=callback_group,
        )
        self.get_logger().info("'/pedestal_remove' 토픽 구독 시작")

        # 준비 완료 신호 발행자
        self.ready_pub = self.create_publisher(Empty, '/pose_target_ready', 10)
        # plan/execute 완료 신호 발행자 (pick_place_node가 구독)
        self.done_pub = self.create_publisher(Bool, '/pose_target_done', 10)
        self._ready_sent = False

        self.ready_check_timer = self.create_timer(
            0.5, self._check_and_send_ready
        )

    def destroy_node(self):
        """소켓을 닫고 노드를 종료한다."""
        self.send_sock.close()
        super().destroy_node()

    def _check_and_send_ready(self):
        """joint_state 수신 확인 후 ready 신호 발행을 예약한다."""
        if self._ready_sent:
            return
        if self.moveit2.joint_state is not None:
            self.get_logger().info(
                "joint_states 확인됨. 3초 후 ready 신호 발행 예정"
            )
            self.ready_check_timer.cancel()
            self.create_timer(8.0, self._send_ready_once)

    def _send_ready_once(self):
        """'/pose_target_ready' 신호를 최초 1회만 발행한다."""
        if self._ready_sent:
            return
        self._ready_sent = True
        self.ready_pub.publish(Empty())
        self.get_logger().info("'/pose_target_ready' 신호 발행 완료")

    def scene_object_pose_callback(self, msg: PoseStamped):
        """물건 위치를 기반으로 가판대 충돌 객체를 MoveIt2 씬에 등록한다."""
        ox = msg.pose.position.x
        oy = msg.pose.position.y
        oz = msg.pose.position.z

        pedestal_depth_half = PEDESTAL_SIZE[1] / 2.0
        object_half = OBJECT_SIZE[0] / 2.0
        shift = pedestal_depth_half - object_half
        pedestal_x = ox + shift
        pedestal_y = oy
        pedestal_top_z = 0.19  # 가판대 상단 고정값 (g_base 기준, 실측)
        pedestal_z = pedestal_top_z - PEDESTAL_SIZE[2] / 2.0

        pedestal_quat = [0.0, 0.0, -0.707107, 0.707107]

        self.moveit2.remove_collision_object(id="pedestal")
        self.moveit2.add_collision_box(
            id="pedestal",
            size=PEDESTAL_SIZE,
            position=[pedestal_x, pedestal_y, pedestal_z],
            quat_xyzw=pedestal_quat,
            frame_id="g_base",
        )
        self.get_logger().info(
            f"가판대 충돌 객체 등록: center=({pedestal_x:.3f},{pedestal_y:.3f},{pedestal_z:.3f}), "
            f"물건 x={ox:.3f} y={oy:.3f} z={oz:.3f}"
        )

    def pedestal_remove_callback(self, _msg: Empty):
        """MoveIt2 씬에서 가판대 충돌 객체를 제거한다."""
        self.moveit2.remove_collision_object(id="pedestal")
        self.get_logger().info("가판대 충돌 객체 제거 (준비자세 복귀 직전)")

    def _plan_execute_and_send(self, position, quat, label, plan_kwargs):
        """plan → execute → joint_state 읽기 → Pi 전송 공통 로직."""
        max_attempts = 3
        succeeded = False
        for attempt in range(1, max_attempts + 1):
            trajectory = self.moveit2.plan(
                position=position,
                quat_xyzw=quat,
                **plan_kwargs,
            )
            if trajectory is not None:
                self.moveit2.execute(trajectory)
                self.moveit2.wait_until_executed()
                self.get_logger().info(f"{label} plan/execute 완료 (시도 {attempt}/{max_attempts})")
                succeeded = True
                break
            self.get_logger().warn(f"{label} 경로 계획 실패, 재시도 ({attempt}/{max_attempts})")

        if not succeeded:
            self.get_logger().warn(f"{label} 모든 재시도 실패. Pi로 전송하지 않음")
            done_msg = Bool()
            done_msg.data = False
            self.done_pub.publish(done_msg)
            return

        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        joint_state = self.moveit2.joint_state
        if joint_state is None:
            self.get_logger().warn("joint_state를 가져오지 못했습니다")
            done_msg = Bool()
            done_msg.data = False
            self.done_pub.publish(done_msg)
            return

        name_to_pos = dict(zip(joint_state.name, joint_state.position))
        try:
            angles_deg = [math.degrees(name_to_pos[n]) for n in self.joint_names]
        except KeyError as e:
            self.get_logger().warn(f"조인트 이름 불일치: {e}")
            done_msg = Bool()
            done_msg.data = False
            self.done_pub.publish(done_msg)
            return

        if all(abs(a) < 1e-6 for a in angles_deg):
            self.get_logger().warn("joint_state가 전부 0 — 0.5초 대기 후 재확인")
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.1)
            joint_state = self.moveit2.joint_state
            name_to_pos = dict(zip(joint_state.name, joint_state.position))
            angles_deg = [math.degrees(name_to_pos[n]) for n in self.joint_names]
            if all(abs(a) < 1e-6 for a in angles_deg):
                self.get_logger().warn("재확인 후에도 전부 0 — Pi 전송 생략")
                done_msg = Bool()
                done_msg.data = False
                self.done_pub.publish(done_msg)
                return

        data = {'angles': angles_deg, 'speed': 30}
        self.send_sock.sendto(json.dumps(data).encode(), ('127.0.0.1', 9999))
        self.get_logger().info(f"Pi 전송 완료: {angles_deg}")

        done_msg = Bool()
        done_msg.data = True
        self.done_pub.publish(done_msg)

    def target_pose_cartesian_callback(self, msg: PoseStamped):
        """직선 경로(cartesian) 목표를 수신하여 plan/execute 후 Pi로 전송한다."""
        self.get_logger().info(
            f"직선 경로 목표 수신: x={msg.pose.position.x:.3f}, "
            f"y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}"
        )
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        quat = [
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        ]
        self._plan_execute_and_send(position, quat, "직선 경로", {
            'tolerance_position': 0.001,
            'tolerance_orientation': 0.1,
            'cartesian': True,
            'cartesian_fraction_threshold': 0.9,
        })

    def target_pose_pickup_callback(self, msg: PoseStamped):
        """픽업 목표(느슨한 오차)를 수신하여 plan/execute 후 Pi로 전송한다."""
        self.get_logger().info(
            f"픽업 목표 수신(느슨한 오차): x={msg.pose.position.x:.3f}, "
            f"y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}"
        )
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        quat = [
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        ]
        self._plan_execute_and_send(position, quat, "픽업", {
            'tolerance_position': 0.005,
            'tolerance_orientation': 0.35,
        })

    def target_pose_callback(self, msg: PoseStamped):
        """일반 목표 좌표를 수신하여 plan/execute 후 Pi로 전송한다."""
        self.get_logger().info(
            f"목표 좌표 수신: x={msg.pose.position.x:.3f}, "
            f"y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}"
        )
        position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        quat = [
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        ]
        self._plan_execute_and_send(position, quat, "", {
            'tolerance_position': 0.001,
            'tolerance_orientation': 0.1,
        })


def main(args=None):
    """pi_sender_node를 서브프로세스로 실행 후 PoseTargetController 노드를 시작한다."""
    sender_script = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        'pi_sender_node.py'
    )
    sender_env = os.environ.copy()
    sender_proc = subprocess.Popen(  # noqa: R1732
        [sys.executable, sender_script],
        env=sender_env
    )
    time.sleep(2)

    rclpy.init(args=args)
    node = PoseTargetController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sender_proc.terminate()


if __name__ == '__main__':
    main()
