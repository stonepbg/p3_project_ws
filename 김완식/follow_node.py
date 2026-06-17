import os
import time
import math
import numpy as np
import cv2
import pyrealsense2 as rs
from ultralytics import YOLO

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

PKG_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(PKG_DIR, "yolo11n.pt")
TRACKER_PATH = os.path.join(PKG_DIR, "botsort_reid.yaml")

# ===== 파라미터 =====
CENTER_X = 320
IMG_WIDTH = 640
HFOV_DEG = 69.0
LOCK_FRAMES = 15          # 워밍업 프레임
SHOW_WINDOW = True
LOST_DELAY = 8            # 사람 사라진 후 이 프레임 지나야 탐색 시작 (과민 방지)
FIND_HOLD_SEC = 1.0       # find(재발견) 시 정지 유지 시간

# linear.z 모드 플래그
MODE_FOLLOW = 0.0
MODE_SEARCH_LEFT = 1.0
MODE_SEARCH_RIGHT = 2.0
MODE_FIND = 3.0
# ====================


# ============================================================
# MJPEG 스트리밍 서버 (follow 로직과 무관, 독립 실행)
# 브라우저: http://192.168.0.103:8080/stream
# ============================================================
_stream_lock = threading.Lock()
_stream_frame = None

class _MjpegHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path != "/stream":
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _stream_lock:
                    frame = _stream_frame
                if frame is not None:
                    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if ok:
                        data = buf.tobytes()
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(data)
                        self.wfile.write(b"\r\n")
                time.sleep(0.1)
        except Exception:
            pass

def _start_stream_server(port=8080):
    srv = ThreadingHTTPServer(("0.0.0.0", port), _MjpegHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
# ============================================================


class FollowNode(Node):
    def __init__(self):
        super().__init__("follow_node")
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

        self.frame_count = 0
        self.started = False
        self.last_seen_side = None    # "left" / "right"
        self.lost_count = 0           # 사람 안 보인 프레임 수
        self.searching = False        # 탐색 중 여부
        self.find_until = 0.0         # find(정지) 유지 종료 시각

        _start_stream_server(8080)
        self.get_logger().info("MJPEG stream: http://192.168.0.103:8080/stream")
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
        cx_err = cx - CENTER_X
        return -(cx_err / IMG_WIDTH) * math.radians(HFOV_DEG)

    def publish(self, mode, distance=0.0, angle=0.0):
        # linear.z=모드, linear.x=거리, angular.z=각도
        msg = Twist()
        msg.linear.x = float(distance)
        msg.linear.z = float(mode)
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

        # 워밍업
        if not self.started:
            if self.frame_count >= LOCK_FRAMES:
                self.started = True
                self.get_logger().info("[START] 추종 시작")
            if SHOW_WINDOW:
                cv2.putText(annotated, f"WARMING UP {self.frame_count}/{LOCK_FRAMES}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            global _stream_frame
            with _stream_lock:
                _stream_frame = annotated.copy()
            if SHOW_WINDOW:
                cv2.imshow("follow", annotated)
                cv2.waitKey(1)
            return

        # 유효한 사람 목록 (depth > 0)
        people = []
        if boxes.id is not None:
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                dist = self.median_depth(depth, cx, cy)
                if dist > 0:
                    people.append((dist, cx, cy, x1, y1, x2, y2))

        distance, angle = 0.0, 0.0
        status = "SEARCHING"
        now = time.time()

        # ===== find(재발견 정지) 유지 중 =====
        if self.find_until > 0.0:
            if now < self.find_until:
                status = "FIND_HOLD"
                self.publish(MODE_FIND)
                if people:
                    people.sort(key=lambda p: p[0])
                    d, cx, cy, x1, y1, x2, y2 = people[0]
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 165, 255), 3)
                self._draw_and_stream(annotated, status, 0.0, 0.0)
                return
            else:
                self.find_until = 0.0
                if people:
                    self.searching = False
                else:
                    self.searching = True

        if people:
            # ===== 추종 =====
            people.sort(key=lambda p: p[0])
            dist, cx, cy, x1, y1, x2, y2 = people[0]

            if self.searching:
                self.find_until = now + FIND_HOLD_SEC
                self.searching = False
                self.get_logger().info("[FIND] 사람 재발견 → 정지")
                self.publish(MODE_FIND)
                status = "FIND_START"
            else:
                distance = dist
                angle = self.pixel_to_angle(cx)
                status = "FOLLOWING"
                self.lost_count = 0
                if cx < CENTER_X - 40:
                    self.last_seen_side = "left"
                elif cx > CENTER_X + 40:
                    self.last_seen_side = "right"
                self.publish(MODE_FOLLOW, distance, angle)

            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 3)
            cv2.putText(annotated, f"TARGET {dist:.2f}m",
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)
        else:
            # ===== 사람 없음 =====
            self.lost_count += 1
            if self.lost_count >= LOST_DELAY:
                self.searching = True
                if self.last_seen_side == "right":
                    status = "SEARCH_RIGHT"
                    self.publish(MODE_SEARCH_RIGHT)
                else:
                    status = "SEARCH_LEFT"
                    self.publish(MODE_SEARCH_LEFT)
            else:
                status = "LOST_WAIT"

        self._draw_and_stream(annotated, status, distance, angle)

    def _draw_and_stream(self, annotated, status, distance, angle):
        global _stream_frame
        with _stream_lock:
            _stream_frame = annotated.copy()
        if SHOW_WINDOW:
            cv2.line(annotated, (CENTER_X, 0), (CENTER_X, 480), (255, 0, 0), 1)
            cv2.putText(annotated, status, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(annotated, f"dist:{distance:.2f}m ang:{angle:+.3f}rad",
                        (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("follow", annotated)
            cv2.waitKey(1)

    def stop_cmd(self):
        try:
            for _ in range(10):
                self.publish(MODE_FIND)  # 정지(find=정지)
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
