import os
import time
import math
import numpy as np
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 패키지 내부 파일 경로
PKG_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(PKG_DIR, "yolo11n.pt")
TRACKER_PATH = os.path.join(PKG_DIR, "botsort_reid.yaml")

# ===== 파라미터 =====
CENTER_X = 320
IMG_WIDTH = 640
HFOV_DEG = 69.0          # D435 RGB 수평 FOV
LOCK_FRAMES = 15
SHOW_WINDOW = True
SEARCH_ANGLE = 0.3       # 타겟 소실 시 탐색용 각도 (rad)
# ====================


class FollowNode(Node):
    def __init__(self):
        super().__init__("follow_node")
        # /target_cmd 발행: linear.x=거리(m), angular.z=각도(rad)
        self.cmd_pub = self.create_publisher(Twist, "/target_cmd", 10)

        self.get_logger().info("loading YOLO...")
        self.model = YOLO(MODEL_PATH)

        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        for _ in range(30):
            self.pipeline.wait_for_frames()

        self.target_id = None
        self.frame_count = 0
        self.last_seen_side = None
        self.relock_candidate = None
        self.relock_count = 0
        self.RELOCK_NEED = 8
        self.get_logger().info("follow_node started. target = nearest person")

        self.timer = self.create_timer(1.0 / 30.0, self.loop)

    def median_depth(self, depth_frame, cx, cy, k=5):
        half = k // 2
        vals = []
        for dx in range(-half, half + 1):
            for dy in range(-half, half + 1):
                d = depth_frame.get_distance(cx + dx, cy + dy)
                if d > 0:
                    vals.append(d)
        return float(np.median(vals)) if vals else 0.0

    def pixel_to_angle(self, cx):
        # cx_err: 화면 중심 대비 픽셀 오차
        # 왼쪽(+), 오른쪽(-) 부호로 라디안 각도 반환
        cx_err = cx - CENTER_X
        angle = -(cx_err / IMG_WIDTH) * math.radians(HFOV_DEG)
        return angle

    def publish_cmd(self, distance, angle):
        # linear.x = 거리(m), angular.z = 각도(rad)
        msg = Twist()
        msg.linear.x = float(distance)
        msg.angular.z = float(angle)
        self.cmd_pub.publish(msg)


    def loop(self):
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            return

        img = np.asanyarray(color.get_data())
        results = self.model.track(
            img, persist=True, classes=[0],
            tracker=TRACKER_PATH, verbose=False)
        boxes = results[0].boxes
        annotated = img.copy()
        self.frame_count += 1

        people = {}
        if boxes.id is not None:
            ids = boxes.id.int().tolist()
            for box, tid in zip(boxes, ids):
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                dist = self.median_depth(depth, cx, cy)
                people[tid] = (cx, cy, dist, (x1, y1, x2, y2))

        if self.target_id is None and self.frame_count >= LOCK_FRAMES and people:
            valid = {t: v for t, v in people.items() if v[2] > 0}
            if valid:
                self.target_id = min(valid, key=lambda t: valid[t][2])
                self.get_logger().info(
                    f"[LOCK] target_id={self.target_id} "
                    f"dist={valid[self.target_id][2]:.2f}m")

        # 발행할 값: distance(m), angle(rad)
        distance, angle = 0.0, 0.0
        status = "SEARCHING"

        if self.target_id is not None and self.target_id in people:
            cx, cy, dist, (x1, y1, x2, y2) = people[self.target_id]
            if dist > 0:
                distance = dist
                angle = self.pixel_to_angle(cx)
                status = "FOLLOWING"
                if cx < CENTER_X - 40:
                    self.last_seen_side = "left"
                elif cx > CENTER_X + 40:
                    self.last_seen_side = "right"
            else:
                status = "TARGET_NO_DEPTH"
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.putText(annotated, f"TARGET ID:{self.target_id} {dist:.2f}m",
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)
        elif self.target_id is not None:
            # 타겟 소실 - 마지막 본 방향으로 탐색 각도 발행 (거리 0)
            status = "SEARCHING_ROTATE"
            distance = 0.0
            if self.last_seen_side == "right":
                angle = -SEARCH_ANGLE
            elif self.last_seen_side == "left":
                angle = SEARCH_ANGLE
            else:
                angle = SEARCH_ANGLE
            if people:
                valid = {t: v for t, v in people.items() if v[2] > 0}
                if valid:
                    cand = min(valid, key=lambda t: valid[t][2])
                    if cand == self.relock_candidate:
                        self.relock_count += 1
                    else:
                        self.relock_candidate = cand
                        self.relock_count = 1
                    if self.relock_count >= self.RELOCK_NEED:
                        self.target_id = cand
                        self.relock_count = 0
                        self.get_logger().info(f"[RELOCK] target_id={self.target_id}")

        self.publish_cmd(distance, angle)

        for tid, (cx, cy, dist, (x1, y1, x2, y2)) in people.items():
            if tid == self.target_id:
                continue
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 1)

        if SHOW_WINDOW:
            cv2.line(annotated, (CENTER_X, 0), (CENTER_X, 480), (255, 0, 0), 1)
            cv2.putText(annotated, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(annotated, f"dist:{distance:.2f}m ang:{angle:+.3f}rad",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("follow", annotated)
            cv2.waitKey(1)

    def stop_cmd(self):
        # 종료 시 정지 의미: 거리 0, 각도 0 여러 번 발행
        try:
            for _ in range(10):
                self.publish_cmd(0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.02)
        except Exception:
            pass

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 - 정지 신호 발행")
        node.stop_cmd()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
