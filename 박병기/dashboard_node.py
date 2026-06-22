#!/usr/bin/env python3
"""
dashboard_node.py (v33 — 3번 화면 [주행], [매니저] 로그 파싱 필터링 해제 및 색상 연동 버전)
──────────────────────────────────────────────────────────────────
구독 토픽 (Sub):
  /traffic_sign_topic       std_msgs/String
  /control_state            std_msgs/String
  /AGV_log                  std_msgs/String  ← 이벤트 드리븐 통합 로그 구독

발행 토픽 (Pub):
  /nano_send_status         std_msgs/Int32   ← 정수형 제어 패킷 발행
──────────────────────────────────────────────────────────────────
"""

import threading
import time
import os
from collections import deque
from datetime import datetime

import cv2
os.environ["ROS_DOMAIN_ID"] = "0"
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from flask import Flask, jsonify, render_template_string, request

OUTPUT_ROOT   = os.path.abspath("dashboard_output")
CAPTURE_DIR   = os.path.join(OUTPUT_ROOT, "captures")
BURST_INTERVAL = 0.5

STREAM_URL_1 = "http://192.168.0.103:8080/stream"
STREAM_URL_2 = "http://192.168.0.103:8081/stream"

HTML = r"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, minimum-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>AGV Remote Management Core</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700&display=swap" rel="stylesheet">
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
<style>
:root {
  --bg:      #06080c;
  --surface: #0b0f17;
  --card:    #0f1420;
  --border:  #1b263b;
  --bhi:     #293d5c;
  --text:    #c5d4e8;
  --dim:     #637d9f;
  --g: #00e5a0;
  --b: #00aaff;
  --y: #ffbe00;
  --r: #ff3355;
  --mono: 'Share Tech Mono', monospace;
  --sans: 'Rajdhani', sans-serif;
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{background:var(--bg);color:var(--text);font-family:var(--sans);width:100%;}

/* PC/태블릿 기본 레이아웃 */
@media (min-width: 1024px) {
  html,body{height:100%;overflow:hidden}
  body{height:100vh;display:flex;flex-direction:column}
  .main{display:grid;grid-template-columns:1fr 450px;gap:14px;padding:12px 20px;flex:1;min-height:0;overflow:hidden;}
  .center-panel{display:flex;flex-direction:column;gap:14px;min-height:0;overflow:hidden;}
  .video-grid{display:grid;grid-template-columns:1fr 1fr;gap:14px;flex:1;min-height:0;}
  .side-panel{display:flex;flex-direction:column;gap:14px;overflow-y:auto;min-height:0}
}

/* ── 헤더 영역 ── */
header{
  display:flex;align-items:center;padding:12px 20px;
  background:var(--surface);border-bottom:2px solid var(--border);flex-shrink:0;
}
.logo-title{font-size:24px;font-weight:700;letter-spacing:1px;color:#fff}
.logo-sub{font-family:var(--mono);font-size:13px;color:var(--dim);margin-left:15px}
.badges{margin-left:auto;display:flex;gap:12px}
.badge{
  font-family:var(--mono);font-size:13px;padding:4px 10px;border-radius:20px;
  border:2px solid var(--border);display:flex;align-items:center;gap:6px;font-weight:700;
}
.badge.live{border-color:var(--g);color:var(--g)}
.badge .dot{width:8px;height:8px;border-radius:50%;background:var(--g)}

/* ── 탑 유틸리티 바 ── */
.top-bar{
  display:flex;gap:12px;padding:10px 20px;background:var(--surface);
  border-bottom:2px solid var(--border);flex-shrink:0;align-items:center;
}
.cap-btn{
  flex:1;max-width:180px;padding:8px 14px;border-radius:8px;border:2px solid var(--bhi);
  background:transparent;color:var(--text);font-family:var(--sans);font-size:16px;
  font-weight:700;cursor:pointer;transition:all .2s;text-align:center;
}
.cap-btn:hover{border-color:#fff;background:rgba(255,255,255,0.05)}
.cap-btn.bursting{border-color:var(--g);color:var(--g);background:rgba(0,229,160,.08)}

/* 컴포넌트 공통 카드 사양 */
.card{background:var(--card);border:2px solid var(--border);border-radius:12px;overflow:hidden;display:flex;flex-direction:column;margin-bottom:2px;flex-shrink:0;}
.card-hd{display:flex;align-items:center;justify-content:space-between;padding:10px 16px;border-bottom:2px solid var(--border);flex-shrink:0}
.card-title{font-size:18px;font-weight:700;letter-spacing:.8px;text-transform:uppercase}

/* 캠 영상 스트리밍 보드 */
.feed-wrap{
  position:relative;background:#000;flex:1;min-height:200px;
  display:flex;align-items:center;justify-content:center;overflow:hidden;
}
.feed-wrap img{width:100%;height:100%;max-height:50vh;object-fit:contain}
.feed-lbl{position:absolute;bottom:8px;left:12px;font-family:var(--mono);font-size:13px;color:rgba(255,255,255,.4);z-index:4}

/* ── 3번 화면 터미널 7라인 고정 설계 ── */
.agv-terminal{
  background:#030508;border-top:2px solid var(--border);
  font-family:var(--mono);padding:12px 14px;overflow-y:auto;font-size:17px;
  line-height:1.5;white-space:pre-wrap;font-weight:600;
  letter-spacing: 0.5px;
  height:210px; 
  flex-shrink:0;
}
.parsed-log-item { margin-bottom: 2px; border-bottom: 1px solid rgba(255,255,255,0.02); padding-bottom: 2px; }
.log-cat-manager { color: #00aaff !important; } 
.log-cat-drive { color: #ffbe00 !important; }   
.log-cat-default { color: #00e5a0 !important; } 

/* ── 제어 패킷 패딩 및 버튼 크기 축소 본 유지 ── */
.ctrl-group{padding:10px 14px;display:flex;flex-direction:column;gap:10px;position:relative;}

.mode-toggle-container {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 10px;
  width: 100%;
  margin-bottom: 2px;
}

.btn-stop, .btn-follow, .btn-action, .btn-dest {
  width:100%; font-family:var(--sans); border-radius:8px; cursor:pointer; transition:all 0.15s;
}

button:disabled {
  opacity: 0.25 !important;
  cursor: not-allowed !allowed !important;
}

button.keep-color:disabled {
  opacity: 1.0 !important;
  box-shadow: 0 0 12px currentColor !important;
}

.btn-stop{padding:12px;min-height:50px;background:rgba(255,51,85,0.14);border:2px solid var(--r);color:var(--r);font-size:21px;font-weight:700}
.btn-stop:hover:not(:disabled){background:var(--r);color:#fff;box-shadow: 0 0 10px var(--r);}

.btn-follow{padding:12px;min-height:50px;background:rgba(0,229,160,0.12);border:2px solid var(--g);color:var(--g);font-size:21px;font-weight:700}
.btn-follow:hover:not(:disabled){background:var(--g);color:#06080c}

.btn-action { padding:11px; min-height:48px; font-size:19px; font-weight:700; background: transparent; border:2px solid var(--border); color: var(--text); }
.btn-action#btn-guide-main { border-color: var(--b); color: var(--b); }
.btn-action#btn-auto-main { border-color: var(--y); color: var(--y); }
.btn-action#btn-guide-main:hover:not(:disabled) { background: rgba(0,170,255,0.1); }
.btn-action#btn-auto-main:hover:not(:disabled) { background: rgba(255,190,0,0.1); }

.dest-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;padding-bottom:2px;}
.btn-dest{padding:11px 8px;min-height:46px;background:transparent;border:2px solid var(--bhi);color:var(--text);font-size:17px;font-weight:700}
.btn-dest:hover:not(:disabled) { border-color: #fff; background: rgba(255, 255, 255, 0.08); }

.target-select-guide {
  display: none;
  font-size: 17px;
  font-weight: 700;
  text-align: center;
  padding: 10px;
  margin: 2px 0 2px 0;
  background: rgba(0, 170, 255, 0.08);
  border: 2px dashed var(--b);
  border-radius: 8px;
  animation: pulse-guide 1.5s infinite;
  width: 100%;
  flex-shrink: 0;
}
.target-select-guide i { margin-right: 6px; }

.active-follow { background: var(--g) !important; color: #06080c !important; }
.active-guide { background: var(--b) !important; color: #fff !important; }
.active-auto { background: var(--y) !important; color: #06080c !important; }

/* ── 발행 토픽 기록 컨테이너 높이 18px 차감 (420px -> 402px) ── */
.log-list-body {
  background: #030508; 
  padding: 14px; 
  height: 402px; 
  overflow-y: auto;
  font-family: var(--mono); 
  font-size: 18px; 
  display: flex; 
  flex-direction: column; 
  gap: 8px;
}
.log-item { display: flex; gap: 10px; border-bottom: 1px dashed rgba(27, 38, 59, 0.4); padding-bottom: 6px; }
.log-time { color: var(--dim); }
.log-msg { color: var(--g); font-weight: 600; }
.log-clear-btn {
  background: transparent; border: 1px solid var(--r); color: var(--r);
  padding: 3px 8px; font-family: var(--sans); font-size: 12px; font-weight: 700;
  border-radius: 4px; cursor: pointer; transition: all 0.15s;
}
.log-clear-btn:hover { background: var(--r); color: #fff; }

@media (max-width: 1023px) {
  body { overflow-y: auto; height: auto; }
  .main { display: flex; flex-direction: column; gap: 14px; padding: 12px; }
  .center-panel, .video-grid, .side-panel { display: flex; flex-direction: column; gap: 14px; }
  header { padding: 12px; flex-wrap: wrap; gap: 8px; }
  .logo-title { font-size: 20px; }
  .logo-sub { font-size: 12px; margin-left: 0; width: 100%; }
  .badges { margin-left: 0; width: 100%; justify-content: flex-start; }
  .top-bar { padding: 8px 12px; }
  .agv-terminal { font-size: 15px; height: 180px; }
  .log-list-body { height: 250px; font-size: 16px; }
}
</style>
</head>
<body>

<header>
  <div class="logo-title">AGV Core Dashboard</div>
  <div class="logo-sub">Multi-Stream & Robotics Control Platform</div>
  <div class="badges"><div class="badge live"><div class="dot"></div>SYSTEM ACTIVE</div></div>
</header>

<div class="top-bar">
  <button class="cap-btn" onclick="captureSingle()">📸 스냅샷</button>
  <button class="cap-btn" id="btn-burst" onclick="toggleBurst()">🔁 연속 캡처</button>
</div>

<div class="main">
  <div class="center-panel">
    <div class="video-grid">
      <div class="card">
        <div class="card-hd"><span class="card-title" style="color:var(--g)">전방 화면 (:8080)</span></div>
        <div class="feed-wrap"><img src="{{ stream_1 }}" alt="Cam 1"><div class="feed-lbl">{{ stream_1 }}</div></div>
      </div>
      <div class="card">
        <div class="card-hd"><span class="card-title" style="color:var(--b)">후방 화면 (:8081)</span></div>
        <div class="feed-wrap"><img src="{{ stream_2 }}" alt="Cam 2"><div class="feed-lbl">{{ stream_2 }}</div></div>
      </div>
    </div>
    <div class="card">
      <div class="card-hd"><span class="card-title" style="color:var(--y)">AVG 로봇 로그 (/AGV_log)</span></div>
      <div class="agv-terminal" id="agv-console">대기 중... 외부 로봇 통합 이벤트 로그를 실시간 대기하고 있습니다.</div>
    </div>
  </div>

  <div class="side-panel">
    <div class="card">
      <div class="card-hd">
        <span class="card-title" style="color:#fff; font-size:20px; display:flex; align-items:center;">
          AGV REMOTE CONTROL
        </span>
      </div>

      <div class="ctrl-group" style="border-bottom:2px solid var(--border)">
        <button id="btn-stop-core" class="btn-stop" onclick="clickStop()">정지</button>
      </div>

      <div class="ctrl-group" style="border-bottom:2px solid var(--border)">
        <button id="btn-follow-core" class="btn-follow" onclick="clickFollow()">추종</button>
      </div>

      <div class="ctrl-group">
        <div class="mode-toggle-container">
          <button id="btn-guide-main" class="btn-action" onclick="toggleMode('guide')">안내</button>
          <button id="btn-auto-main" class="btn-action" onclick="toggleMode('auto')">자율주행</button>
        </div>

        <div class="dest-grid">
          <button id="dest-0" class="btn-dest" onclick="clickDest(0)" disabled>목적지 0</button>
          <button id="dest-1" class="btn-dest" onclick="clickDest(1)" disabled>목적지 1</button>
          <button id="dest-2" class="btn-dest" onclick="clickDest(2)" disabled>목적지 2</button>
          <button id="dest-3" class="btn-dest" onclick="clickDest(3)" disabled>목적지 3</button>
          <button id="dest-4" class="btn-dest" onclick="clickDest(4)" disabled>목적지 4</button>
          <button id="dest-5" class="btn-dest" onclick="clickDest(5)" disabled>목적지 5</button>
          <button id="dest-6" class="btn-dest" onclick="clickDest(6)" disabled>목적지 6</button>
          <button id="dest-7" class="btn-dest" onclick="clickDest(7)" disabled>목적지 7</button>
        </div>
      </div>
    </div>

    <div id="text-dest-guide" class="target-select-guide">
      <i class="fa-regular fa-hand-pointer"></i> 목적지를 선택 해 주세요!
    </div>

    <div class="card">
      <div class="card-hd">
        <span class="card-title" style="font-size:16px; color:var(--b)">발행 토픽 기록 (CMD LOG)</span>
        <button class="log-clear-btn" onclick="clearLog()">CLEAR</button>
      </div>
      <div class="log-list-body" id="log-container">
        <div style="color:var(--dim); font-style:italic;">발행된 원격 명령 토픽 내역이 없습니다.</div>
      </div>
    </div>

  </div>
</div>

<script>
let currentActiveMode = null; 
let previousLogData = ""; 

function toggleMode(mode) {
  currentActiveMode = mode;
  const guideBtn = document.getElementById('btn-guide-main');
  const autoBtn = document.getElementById('btn-auto-main');
  const guideText = document.getElementById('text-dest-guide');

  guideText.style.display = 'block';

  if (mode === 'guide') {
    guideBtn.classList.add('active-guide');
    autoBtn.classList.remove('active-auto');
  } else if (mode === 'auto') {
    autoBtn.classList.add('active-auto');
    guideBtn.classList.remove('active-guide');
  }

  const destButtons = document.querySelectorAll('.btn-dest');
  destButtons.forEach(btn => { btn.disabled = false; });
}

function clickDest(num) {
  if (!currentActiveMode) return;

  let finalTopicValue = 0;
  let logLabel = '';
  let activeMainId = '';

  if (currentActiveMode === 'guide') {
    finalTopicValue = 20 + num; 
    logLabel = `안내 - 목적지 ${num}`;
    activeMainId = 'btn-guide-main';
  } else {
    finalTopicValue = 30 + num; 
    logLabel = `자율주행 - 목적지 ${num}`;
    activeMainId = 'btn-auto-main';
  }

  pubCmd(finalTopicValue);
  addLogRecord(finalTopicValue, logLabel);

  const targetDestId = 'dest-' + num;
  const targetDestBtn = document.getElementById(targetDestId);
  
  if (currentActiveMode === 'guide') {
    targetDestBtn.classList.add('active-guide');
  } else {
    targetDestBtn.classList.add('active-auto');
  }

  document.getElementById('text-dest-guide').style.display = 'none';

  const allUiButtons = document.querySelectorAll('button:not(.cap-btn):not(.log-clear-btn)');
  allUiButtons.forEach(btn => {
    if (btn.id === 'btn-stop-core') {
      btn.disabled = false;
    } else if (btn.id === activeMainId || btn.id === targetDestId) {
      btn.disabled = true;  
      btn.classList.add('keep-color'); 
    } else {
      btn.disabled = true;  
    }
  });
}

function clickFollow() {
  pubCmd(10);
  addLogRecord(10, '추종');
  document.getElementById('text-dest-guide').style.display = 'none';
  
  const followBtn = document.getElementById('btn-follow-core');
  followBtn.classList.add('active-follow');

  const allUiButtons = document.querySelectorAll('button:not(.cap-btn):not(.log-clear-btn)');
  allUiButtons.forEach(btn => {
    if (btn.id === 'btn-stop-core') {
      btn.disabled = false;
    } else if (btn.id === 'btn-follow-core') {
      btn.disabled = true;
      btn.classList.add('keep-color');
    } else {
      btn.disabled = true;
    }
  });
}

function clickStop() {
  currentActiveMode = null;
  pubCmd(0);
  addLogRecord(0, '정지');
  document.getElementById('text-dest-guide').style.display = 'none';

  const allUiButtons = document.querySelectorAll('button:not(.cap-btn):not(.log-clear-btn)');
  allUiButtons.forEach(btn => {
    if (btn.classList.contains('btn-dest')) {
      btn.disabled = true;
    } else {
      btn.disabled = false;
    }
    btn.classList.remove('keep-color', 'active-follow', 'active-guide', 'active-auto');
  });
}

function addLogRecord(numericVal, labelText) {
  const container = document.getElementById('log-container');
  if (container.children.length === 1 && container.children[0].style.fontStyle === 'italic') {
    container.innerHTML = '';
  }
  
  const now = new Date();
  const timeStr = `[${now.getHours().toString().padStart(2,'0')}:${now.getMinutes().toString().padStart(2,'0')}:${now.getSeconds().toString().padStart(2,'0')}]`;
  
  const logItem = document.createElement('div');
  logItem.className = 'log-item';
  logItem.innerHTML = `
    <span class="log-time">${timeStr}</span>
    <span class="log-msg" style="color:var(--y)">→ ${numericVal} <span style="color:var(--dim); font-size:15px; font-weight:normal;">(${labelText})</span></span>
  `;
  
  container.appendChild(logItem);
  container.scrollTop = container.scrollHeight;
}

function clearLog() {
  document.getElementById('log-container').innerHTML = '<div style="color:var(--dim); font-style:italic;">발행된 원격 명령 토픽 내역이 없습니다.</div>';
}

async function pubCmd(val) {
  try {
    await fetch('/api/pub/nano_status', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({command: parseInt(val, 10)})
    });
  } catch(e) { console.error("통신 연동 실패", e); }
}

async function captureSingle() { try { await fetch('/api/capture/single', {method:'POST'}); } catch(e){} }
async function toggleBurst() {
  try {
    const res = await fetch('/api/burst/toggle', {method:'POST'});
    const d = await res.json();
    document.getElementById('btn-burst').classList.toggle('bursting', d.bursting);
  } catch(e){}
}

/* ── [수정 및 버그 해결] [매니저], [주행] 문구 필터링 제거 및 정상 렌더링 매핑 ── */
function renderParsedLog(rawText) {
  if (!rawText) return "대기 중... 외부 로봇 통합 이벤트 로그를 실시간 대기하고 있습니다.";
  const lines = rawText.split('\n');
  let resultHtml = "";
  
  lines.forEach(line => {
    if (!line.trim()) return;
    
    let colorClass = "log-cat-default";
    
    // [매니저] 로그 필터링 제거 및 전용 파란색 색상 매핑
    if (line.includes("[매니저]")) {
      colorClass = "log-cat-manager";
    } else if (line.includes("[주행]")) {
      colorClass = "log-cat-drive";
    } else if (line.includes("[추종]")) {
      colorClass = "log-cat-default";
    }
    
    resultHtml += `<div class="parsed-log-item ${colorClass}">${line}</div>`;
  });
  
  if (!resultHtml) return "대기 중... 외부 로봇 통합 이벤트 로그를 실시간 대기하고 있습니다.";
  return resultHtml;
}

async function updateLoop() {
  try {
    const d = await (await fetch('/api/status')).json();
    const con = document.getElementById('agv-console');
    if (d.agv_log !== previousLogData) {
      previousLogData = d.agv_log;
      con.innerHTML = renderParsedLog(d.agv_log);
      con.scrollTop = con.scrollHeight;
    }
  } catch(e){}
  setTimeout(updateLoop, 200);
}
updateLoop();
</script>
</body>
</html>
"""

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        self._lock = threading.Lock()

        self._sign = "UNKNOWN"
        self._brain_state = "--"
        self._brain_cmd = "--"
        
        self._agv_log_buffer = deque(maxlen=100)
        self._agv_log_buffer.append("[시스템] 🟢 대시보드 코어 로깅 허브 노드가 활성화되었습니다.")

        self._bursting = False
        self._burst_thread = None
        self._capture_seq = 0

        os.makedirs(CAPTURE_DIR, exist_ok=True)

        self.create_subscription(String, 'traffic_sign_topic', self._cb_sign, 10)
        self.create_subscription(String, '/control_state', self._cb_brain, 10)
        self.create_subscription(String, '/AGV_log', self._cb_agv_log, 10)

        self._pub_nano = self.create_publisher(Int32, 'nano_send_status', 10)

        self.get_logger().info('대시보드 노드(v33) 로깅 필터링 버그 수정 완료 -> http://localhost:5000')

    def _cb_sign(self, msg):
        with self._lock: self._sign = msg.data

    def _cb_brain(self, msg):
        parts = msg.data.split('|', 1)
        with self._lock:
            self._brain_state = parts[0] if parts else "--"
            self._brain_cmd   = parts[1] if len(parts) > 1 else "--"

    def _cb_agv_log(self, msg):
        now = datetime.now()
        time_prefix = now.strftime("[%H:%M:%S] ")
        with self._lock:
            self._agv_log_buffer.append(f"{time_prefix}{msg.data}")

    def publish_nano_command(self, cmd_int):
        msg = Int32()
        msg.data = cmd_int
        self._pub_nano.publish(msg)
        self.get_logger().info(f'[ROS2 PUB] /nano_send_status -> {cmd_int}')

    def _save_snapshot(self):
        try:
            cap = cv2.VideoCapture(STREAM_URL_1)
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None:
                with self._lock:
                    self._capture_seq += 1
                    seq = self._capture_seq
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                cv2.imwrite(os.path.join(CAPTURE_DIR, f"snapshot_{ts}_{seq:05d}.jpg"), frame)
        except Exception:
            pass

    def capture_single(self):
        threading.Thread(target=self._save_snapshot, daemon=True).start()
        return True

    def toggle_burst(self):
        if self._bursting:
            self._bursting = False
        else:
            self._bursting = True
            self._burst_thread = threading.Thread(target=self._burst_loop, daemon=True)
            self._burst_thread.start()
        return self._bursting

    def _burst_loop(self):
        while self._bursting:
            self._save_snapshot()
            time.sleep(BURST_INTERVAL)

    def get_status(self):
        with self._lock:
            joined_logs = "\n".join(self._agv_log_buffer)
            return {
                "sign": self._sign,
                "brain_state": self._brain_state,
                "brain_cmd": self._brain_cmd,
                "agv_log": joined_logs,
                "bursting": self._bursting
            }


def create_app(node):
    app = Flask(__name__)

    @app.route('/')
    def index():
        return render_template_string(HTML, stream_1=STREAM_URL_1, stream_2=STREAM_URL_2)

    @app.route('/api/status')
    def api_status():
        return jsonify(node.get_status())

    @app.route('/api/pub/nano_status', methods=['POST'])
    def api_pub_nano():
        req_data = request.get_json() or {}
        cmd_val = req_data.get('command', 0)
        node.publish_nano_command(int(cmd_val))
        return jsonify({"status": "success", "published": cmd_val})

    @app.route('/api/capture/single', methods=['POST'])
    def api_capture_single():
        return jsonify({"result": node.capture_single()})

    @app.route('/api/burst/toggle', methods=['POST'])
    def api_burst_toggle():
        return jsonify({"bursting": node.toggle_burst()})

    return app


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    app  = create_app(node)
    
    threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False, threaded=True),
        daemon=True
    ).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()