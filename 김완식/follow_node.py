import os
import time
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

# ===== 제어 파라미터 =====
TARGET_DIST = 1.0
DIST_DEADZONE = 0.15
CENTER_X = 320
CX_DEADZONE = 40
MAX_LINEAR = 0.08
MAX_ANGULAR = 0.3
KP_LINEAR = 0.4
KP_ANGULAR = 0.003
LOCK_FRAMES = 15
SHOW_WINDOW = True   # 모니터에 창 표시 (False면 헤드리스)
# =========================


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FollowNode(Node):
    def __init__(self):
        super().__init__("follow_node")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

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
        self.search_angular = 0.15
        self.relock_candidate = None
        self.relock_count = 0
        self.RELOCK_NEED = 8  # 연속 8프레임 보여야 락
        self.get_logger().info("follow_node started. target = nearest person")

        # 30Hz 루프
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

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
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

        linear, angular = 0.0, 0.0
        status = "SEARCHING"

        if self.target_id is not None and self.target_id in people:
            cx, cy, dist, (x1, y1, x2, y2) = people[self.target_id]
            if dist > 0:
                dist_err = dist - TARGET_DIST
                if abs(dist_err) > DIST_DEADZONE:
                    linear = clamp(KP_LINEAR * dist_err, -MAX_LINEAR, MAX_LINEAR)
                cx_err = cx - CENTER_X
                if abs(cx_err) > CX_DEADZONE:
                    angular = clamp(-KP_ANGULAR * cx_err, -MAX_ANGULAR, MAX_ANGULAR)
                status = "FOLLOWING"
                if cx < CENTER_X - CX_DEADZONE:
                    self.last_seen_side = "left"
                elif cx > CENTER_X + CX_DEADZONE:
                    self.last_seen_side = "right"
            else:
                status = "TARGET_NO_DEPTH"
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.putText(annotated, f"TARGET ID:{self.target_id} {dist:.2f}m",
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)
        elif self.target_id is not None:
            status = "SEARCHING_ROTATE"
            linear = 0.0  # 탐색 중 전진 금지 (제자리 회전만)
            if self.last_seen_side == "right":
                angular = -self.search_angular
            elif self.last_seen_side == "left":
                angular = self.search_angular
            else:
                angular = self.search_angular  # 방향 기록 없으면 기본 좌회전
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

        # 타겟 잃거나 없으면 정지값 발행
        self.publish_cmd(linear, angular)

        for tid, (cx, cy, dist, (x1, y1, x2, y2)) in people.items():
            if tid == self.target_id:
                continue
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 1)

        if SHOW_WINDOW:
            cv2.line(annotated, (CENTER_X, 0), (CENTER_X, 480), (255, 0, 0), 1)
            cv2.putText(annotated, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(annotated, f"lin:{linear:+.3f} ang:{angular:+.3f}",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("follow", annotated)
            cv2.waitKey(1)

    def stop_robot(self):
        # 정지 명령을 여러 번 발행하고 실제 전송 시간 확보
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
        node.get_logger().info("종료 - 로봇 정지")
        node.stop_robot()  # 컨텍스트 살아있을 때 정지 명령
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
jon8g@jon8g-desktop:~/follow_ws/src/follow_tracker/follow_tracker$