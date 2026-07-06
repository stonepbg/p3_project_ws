import time
import os
import subprocess
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String, Float32
from geometry_msgs.msg import Twist, Point, PoseStamped

WIFI_IFACE = "wlP1p1s0"

# 이벤트성 토픽: (토픽명, 타입, 노드, 작업그룹)
EVENT_TOPICS = [
    ("/nano_send_status", Int32, "외부", "모드/제어"),
    ("/agv_mode", Int32, "fsm", "모드/제어"),
    ("/AGV_mode_ack", Int32, "agv", "모드/제어"),
    ("/AGV_status", Int32, "agv", "모드/제어"),
    ("/tts_text", String, "fsm", "모드/제어"),
    ("/follow_enable", Bool, "fsm", "추종"),
    ("/guide_pause", Bool, "fsm", "안내"),
    ("/rear_person", Bool, "follow", "안내"),
    ("/selected_block", Point, "follow", "픽업-블록인식"),
    ("/selected_block_class", String, "follow", "픽업-블록인식"),
    ("/laser_point", Point, "follow", "픽업-블록인식"),
    ("/pickup_active", Bool, "fsm", "픽업-도킹/파지"),
    ("/pickup_target", Point, "pickup", "픽업-도킹/파지"),
    ("/object_pose", PoseStamped, "pickup", "픽업-도킹/파지"),
    ("/ARM_status", Int32, "arm", "픽업-도킹/파지"),
]

# 스트리밍 토픽
TARGET_CMD = "/target_cmd"
CAM_HEALTH = "/cam_health"

TARGET_HZ_OK = (7.0, 30.0)
CAM_FPS_OK = 8.0

PING_TARGETS = [
    ("대시보드", "192.168.0.125"),
    ("AGV", "192.168.0.101"),
    ("로봇팔", "192.168.0.102"),
]
TARGET_EXPECT_HZ = 10.0
# ANSI 색상
C_RESET = "\033[0m"
C_DIM = "\033[90m"
C_RED = "\033[91m"
C_GREEN = "\033[92m"
C_YELLOW = "\033[93m"
C_BLUE = "\033[94m"
C_MAGENTA = "\033[95m"
C_CYAN = "\033[96m"
C_WHITE = "\033[97m"
# 노드별 색 (노드+토픽 동일색으로 구분)
NODE_COLOR = {
    "외부": C_WHITE,
    "agv": C_YELLOW,
    "fsm": C_CYAN,
    "follow": C_GREEN,
    "pickup": C_MAGENTA,
    "arm": "\033[38;5;208m",  # 주황
}
HEARTBEAT_NODES = ["follow_node"]  # heartbeat 발행 노드 목록
HEARTBEAT_TIMEOUT = 6.0            # 이 시간 내 미수신 시 DEAD 판정


class CommMonitor(Node):
    def __init__(self):
        super().__init__("comm_monitor_node")

        # 이벤트성: 마지막 수신시각 + 누적횟수
        self.ev_last = {t[0]: None for t in EVENT_TOPICS}
        self.ev_count = {t[0]: 0 for t in EVENT_TOPICS}
        self.ev_val = {t[0]: None for t in EVENT_TOPICS}
        for name, msgtype, _node, _grp in EVENT_TOPICS:
            self.create_subscription(
                msgtype, name,
                lambda msg, n=name: self._on_event(n, msg), 10)

        # /target_cmd: 수신 시각 기록 -> Hz/지터 계산
        self.tc_follow = []
        self.tc_pickup = []
        self.create_subscription(
            Twist, TARGET_CMD, self._on_target_cmd, 10)

        # /cam_health: 마지막 fps 값 + 수신시각
        self.cam_fps = None
        self.cam_last = None
        self.create_subscription(
            Float32, CAM_HEALTH, self._on_cam_health, 10)
        # /heartbeat/<node>: 노드별 자체 상태 (JSON)
        self.hb_data = {n: None for n in HEARTBEAT_NODES}
        self.hb_last = {n: None for n in HEARTBEAT_NODES}
        for n in HEARTBEAT_NODES:
            self.create_subscription(
                String, f"/heartbeat/{n}",
                lambda msg, nn=n: self._on_heartbeat(nn, msg), 10)

        # CPU 사용률 계산용 직전값
        self._prev_cpu = self._read_cpu_raw()

        # ping 결과 캐시: name -> (loss%, avg_ms, mdev_ms) or None
        self.ping_result = {name: None for name, _ in PING_TARGETS}
        self._ping_stop = False
        self._ping_thread = threading.Thread(target=self._ping_loop, daemon=True)
        self._ping_thread.start()

        self.create_timer(1.0, self.print_status)

    def _on_event(self, name, msg):
        self.ev_last[name] = time.time()
        self.ev_count[name] += 1
        self.ev_val[name] = self._fmt_msg(msg)

    def _fmt_msg(self, msg):
        # 타입별 값 문자열화 (Int32/String/Bool: data / Point: xyz / PoseStamped: pos xyz)
        if hasattr(msg, "data"):
            return str(msg.data)
        if hasattr(msg, "x") and hasattr(msg, "z"):
            return f"x={msg.x:.3f} y={msg.y:.3f} z={msg.z:.3f}"
        if hasattr(msg, "pose"):
            p = msg.pose.position
            return f"x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}"
        return "?" 

    def _on_target_cmd(self, msg):
        now = time.time()
        mode = self.ev_val.get("/agv_mode")
        cat = None
        try:
            cat = int(mode) // 10
        except (TypeError, ValueError):
            cat = None
        if cat == 5:
            self.tc_pickup.append(now)
        else:
            self.tc_follow.append(now)
        # 최근 3초치만 유지 (_target_stats 윈도우와 동일)
        cut = now - 3.0
        self.tc_follow = [t for t in self.tc_follow if t >= cut]
        self.tc_pickup = [t for t in self.tc_pickup if t >= cut]

    def _on_cam_health(self, msg):
        self.cam_fps = msg.data
        self.cam_last = time.time()

    # ---------- 시스템 지표 ----------
    def _on_heartbeat(self, name, msg):
        import json
        try:
            self.hb_data[name] = json.loads(msg.data)
            self.hb_last[name] = time.time()
        except Exception:
            pass

    def _read_cpu_raw(self):
        try:
            with open("/proc/stat") as f:
                parts = f.readline().split()[1:]
            vals = list(map(int, parts))
            idle = vals[3] + vals[4]
            total = sum(vals)
            return idle, total
        except Exception:
            return None

    def cpu_percent(self):
        cur = self._read_cpu_raw()
        if not cur or not self._prev_cpu:
            self._prev_cpu = cur
            return None
        idle0, total0 = self._prev_cpu
        idle1, total1 = cur
        self._prev_cpu = cur
        dt = total1 - total0
        di = idle1 - idle0
        if dt <= 0:
            return None
        return 100.0 * (1.0 - di / dt)

    def mem_percent(self):
        try:
            info = {}
            with open("/proc/meminfo") as f:
                for line in f:
                    k, v = line.split(":")
                    info[k] = int(v.strip().split()[0])
            total = info["MemTotal"]
            avail = info["MemAvailable"]
            return 100.0 * (1.0 - avail / total)
        except Exception:
            return None

    def wifi_rssi(self):
        try:
            with open("/proc/net/wireless") as f:
                for line in f:
                    if WIFI_IFACE in line:
                        cols = line.split()
                        link = cols[2].rstrip(".")
                        level = cols[3].rstrip(".")
                        return int(float(link)), int(float(level))
        except Exception:
            pass
        return None, None

    def fsm_alive(self):
        try:
            names = self.get_node_names()
            return "fsm_node" in names
        except Exception:
            return False

    # ---------- 출력 ----------
    def _pad(self, text, width):
        import unicodedata
        w = sum(2 if unicodedata.east_asian_width(c) in ('W', 'F') else 1 for c in text)
        return text + ' ' * max(0, width - w)

    def print_status(self):
        os.system("clear")
        now = time.time()
        lines = []
        lines.append("=== ROS2 통신 모니터 ===")
        lines.append("")

        # 이벤트성
        lines.append(C_WHITE + "📡 [이벤트 토픽]" + C_RESET + "\n  " + C_DIM + self._pad("노드", 10) + self._pad("토픽", 24) + self._pad("수신", 12) + self._pad("마지막", 10) + self._pad("누적", 6) + "값" + C_RESET)
        prev_grp = None
        for name, _mt, node, grp in EVENT_TOPICS:
            if grp != prev_grp:
                lines.append("  " + C_WHITE + "▶ " + grp + C_RESET)
                prev_grp = grp
            last = self.ev_last[name]
            cnt = self.ev_count[name]
            col = NODE_COLOR.get(node, C_WHITE)
            nodecol = col + self._pad("[" + node + "]", 10) + self._pad(name, 24) + C_RESET
            if last is None:
                lines.append("  " + nodecol + C_DIM + self._pad("⚪미수신", 12) + self._pad("-", 10) + self._pad("x" + str(cnt), 6) + C_RESET)
            else:
                ago = now - last
                fresh = ago <= 30.0
                tag = (C_GREEN + "🟢OK" if fresh else C_YELLOW + "🟡과거") + C_RESET
                val = self.ev_val.get(name)
                lines.append("  " + nodecol + self._pad(tag + " ", 12 + len(C_GREEN) + len(C_RESET)) + self._pad(f"{ago:.1f}s전", 10) + self._pad("x" + str(cnt), 6) + C_WHITE + f"val={val}" + C_RESET)
        lines.append("")

        # target_cmd
        lines.append(C_WHITE + "📶 [스트리밍]" + C_RESET)
        gc = NODE_COLOR["follow"]
        pc = NODE_COLOR["pickup"]
        for label, times, col in [("추종 target_cmd", self.tc_follow, gc), ("픽업 target_cmd", self.tc_pickup, pc)]:
            stats = self._target_stats(times)
            if stats[0] is None:
                lines.append("  " + col + self._pad(label, 18) + C_RESET + C_DIM + "⚪미수신" + C_RESET)
            else:
                hz, jitter, jpct = stats
                ok = TARGET_HZ_OK[0] <= hz <= TARGET_HZ_OK[1]
                tag = (C_GREEN + "🟢OK" if ok else C_RED + "🔴!!") + C_RESET
                dev = (hz - TARGET_EXPECT_HZ) / TARGET_EXPECT_HZ * 100.0
                if jpct < 10.0:
                    jt = C_GREEN + "좋음" + C_RESET
                elif jpct < 25.0:
                    jt = C_YELLOW + "보통" + C_RESET
                else:
                    jt = C_RED + "나쁨" + C_RESET
                lines.append("  " + col + self._pad(label, 18) + C_RESET + tag + f"  {hz:4.1f}Hz (편차 {dev:+4.1f}%)  지터 {jpct:4.1f}% [" + jt + "]")

        # cam_health
        if self.cam_fps is None:
            lines.append("  " + gc + self._pad(CAM_HEALTH, 18) + C_RESET + C_DIM + "⚪미수신" + C_RESET)
        else:
            cam_ago = now - self.cam_last if self.cam_last else 999
            ok = self.cam_fps >= CAM_FPS_OK and cam_ago < 3.0
            tag = (C_GREEN + "🟢OK" if ok else C_RED + "🔴!!") + C_RESET
            lines.append("  " + gc + self._pad(CAM_HEALTH, 18) + C_RESET + tag + f"  {self.cam_fps:4.1f}fps ({cam_ago:.1f}s전)")
        lines.append("")

        # 시스템
        lines.append(C_WHITE + "💓 [하트비트]" + C_RESET)
        now_hb = time.time()
        for n in HEARTBEAT_NODES:
            d = self.hb_data.get(n)
            last = self.hb_last.get(n)
            ncol = NODE_COLOR.get(n.replace("_node", ""), C_WHITE)
            head = ncol + self._pad(n, 16) + C_RESET
            if d is None or last is None:
                lines.append("  " + head + C_DIM + "⚪미수신" + C_RESET)
            elif now_hb - last > HEARTBEAT_TIMEOUT:
                lines.append("  " + head + C_RED + f"🔴DEAD ({now_hb - last:.1f}s 무응답)" + C_RESET)
            else:
                fps = d.get("fps", "-")
                st = d.get("status", "-")
                lines.append("  " + head + C_GREEN + "🟢OK" + C_RESET + f"  fps={fps}  {st}")
        lines.append("")
        lines.append(C_WHITE + "🖥️ [시스템]" + C_RESET)
        cpu = self.cpu_percent()
        mem = self.mem_percent()
        link, level = self.wifi_rssi()
        def _uc(v):
            if v is None:
                return C_DIM + "  -" + C_RESET
            c = C_GREEN if v < 60 else (C_YELLOW if v < 85 else C_RED)
            return c + f"{v:4.1f}%" + C_RESET
        lines.append("  CPU " + _uc(cpu) + "   MEM " + _uc(mem))
        if level is not None:
            if level >= -50:
                wtag, wc = "좋음", C_GREEN
            elif level >= -60:
                wtag, wc = "양호", C_GREEN
            elif level >= -70:
                wtag, wc = "보통", C_YELLOW
            else:
                wtag, wc = "나쁨", C_RED
            lines.append("  📡 WiFi(" + WIFI_IFACE + ")  link " + str(link) + "  RSSI " + str(level) + "dBm  " + wc + "[" + wtag + "]" + C_RESET)
        else:
            lines.append("  📡 WiFi(" + WIFI_IFACE + ")  " + C_DIM + "-" + C_RESET)
        lines.append("")
        lines.append(C_WHITE + "🌐 [네트워크 ping]" + C_RESET)
        for name, ip in PING_TARGETS:
            res = self.ping_result.get(name)
            head = self._pad(name, 8) + C_DIM + "(" + ip + ")" + C_RESET
            if res is None:
                lines.append("  " + head + "  " + C_RED + "🔴응답없음" + C_RESET)
            else:
                loss, avg, mdev = res
                loss_s = f"{loss:.0f}%" if loss is not None else "-"
                avg_s = f"{avg:.1f}ms" if avg is not None else "-"
                mdev_s = f"{mdev:.1f}ms" if mdev is not None else "-"
                ok = (loss == 0.0 and avg is not None)
                tag = (C_GREEN + "🟢OK" if ok else C_RED + "🔴!!") + C_RESET
                lines.append("  " + head + "  " + tag + f"  loss {loss_s}  RTT {avg_s} (±{mdev_s})")
        lines.append("")
        alive = self.fsm_alive()
        acol = C_GREEN if alive else C_RED
        aico = "🟢" if alive else "🔴"
        lines.append("  " + acol + aico + "fsm_node: " + ("살아있음" if alive else "없음") + C_RESET)

        print("\n".join(lines))

    def _ping_loop(self):
        while not self._ping_stop:
            for name, ip in PING_TARGETS:
                self.ping_result[name] = self._ping_once(ip)
            time.sleep(3.0)

    def _ping_once(self, ip):
        try:
            out = subprocess.run(
                ["ping", "-c", "2", "-w", "3", ip],
                capture_output=True, text=True, timeout=5)
            if out.returncode != 0:
                return None
            txt = out.stdout
            loss = None
            avg = None
            mdev = None
            for line in txt.splitlines():
                if "packet loss" in line:
                    seg = line.split("%")[0].split(",")[-1].strip()
                    loss = float(seg)
                if "rtt" in line or "round-trip" in line:
                    nums = line.split("=")[1].strip().split()[0]
                    parts = nums.split("/")
                    avg = float(parts[1])
                    if len(parts) >= 4:
                        mdev = float(parts[3])
            return (loss, avg, mdev)
        except Exception:
            return None

    def _target_stats(self, times):
        now = time.time()
        cut = now - 3.0
        times[:] = [t for t in times if t >= cut]
        ts = sorted(times)
        if len(ts) < 2:
            return None, None
        span = ts[-1] - ts[0]
        if span <= 0:
            return None, None
        hz = (len(ts) - 1) / span
        # 지터: 간격의 표준편차
        gaps = [ts[i+1] - ts[i] for i in range(len(ts)-1)]
        mean = sum(gaps) / len(gaps)
        var = sum((g - mean) ** 2 for g in gaps) / len(gaps)
        jitter = var ** 0.5
        jitter_pct = (jitter / mean * 100.0) if mean > 0 else 0.0
        return hz, jitter, jitter_pct


def main():
    rclpy.init()
    node = CommMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
