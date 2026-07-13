#!/usr/bin/env python3
"""
dashboard_node.py (버튼 높이 5% 확장 + ARM_status 연동 및 픽업 예외 처리 버전)
──────────────────────────────────────────────────────────────────
"""

import threading
import time
import os
import uuid
from collections import deque
from datetime import datetime

import cv2

# ROS 2 미들웨어 설정 및 CycloneDDS 기본 로깅 상태 유지
os.environ["ROS_DOMAIN_ID"] = "0"
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp" 

if "CYCLONEDDS_URI" in os.environ:
    del os.environ["CYCLONEDDS_URI"]

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from flask import Flask, jsonify, render_template_string, request

OUTPUT_ROOT   = os.path.abspath("dashboard_output")
CAPTURE_DIR   = os.path.join(OUTPUT_ROOT, "captures")
MOVIE_DIR   = os.path.join(OUTPUT_ROOT, "movies")
BURST_INTERVAL = 0.5

STREAM_URL_1 = "http://192.168.0.103:8080/stream"
STREAM_URL_2 = "http://192.168.0.101:8080/stream"

HTML = r"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, minimum-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>AGV Remote Management Core</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@400;600;700;800&display=swap" rel="stylesheet">
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
  --p: #E593FF;
  --navy: #1e293b;
  --mono: 'Share Tech Mono', monospace;
  --sans: 'Rajdhani', sans-serif;
}
*{box-sizing:border-box;margin:0;padding:0}
html,body{background:var(--bg);color:var(--text);font-family:var(--sans);width:100%;}

@media (min-width: 1024px) {
  html,body{height:100%;overflow:hidden}
  body{height:100vh;display:flex;flex-direction:column}
  .main{display:grid;grid-template-columns:1fr 365px;gap:14px;padding:12px 20px;flex:1;min-height:0;overflow:hidden;}
  .center-panel{display:flex;flex-direction:column;justify-content: space-between;height: 100%;min-height:0;overflow:hidden;}
  .video-grid{display:grid;grid-template-columns:1fr 1fr;gap:14px;width:100%;flex:1;min-height:0;margin-bottom: 14px;} 
  .side-panel{display:flex;flex-direction:column;gap:14px;height: 100%;overflow:hidden;min-height:0;}
}

header{display:flex;align-items:center;padding:12px 20px;background:var(--surface);border-bottom:2px solid var(--border);flex-shrink:0;}
.logo-title{font-size:24px;font-weight:700;letter-spacing:1px;color:#fff}
.logo-sub{font-family:var(--mono);font-size:13px;color:var(--dim);margin-left:15px}
.badges{margin-left:auto;display:flex;gap:12px}
.badge{font-family:var(--mono);font-size:13px;padding:4px 10px;border-radius:20px;border:2px solid var(--border);display:flex;align-items:center;gap:6px;font-weight:700;}
.badge.live{border-color:var(--g);color:var(--g)}
.badge .dot{width:8px;height:8px;border-radius:50%;background:var(--g)}

.top-bar{display:flex;gap:12px;padding:10px 20px;background:var(--surface);border-bottom:2px solid var(--border);flex-shrink:0;align-items:center;}
.cap-btn{flex:1;max-width:180px;padding:8px 14px;border-radius:8px;border:2px solid var(--bhi);background:transparent;color:var(--text);font-family:var(--sans);font-size:16px;font-weight:700;cursor:pointer;transition:all .2s;text-align:center;}
.cap-btn:hover{border-color:#fff;background:rgba(255,255,255,0.05)}
.cap-btn.bursting{border-color:var(--g);color:var(--g);background:rgba(0,229,160,.08)}

.card{background:var(--card);border:2px solid var(--border);border-radius:12px;overflow:hidden;display:flex;flex-direction:column;width:100%;}
.card-hd{display:flex;align-items:center;justify-content:space-between;padding:12px 16px;border-bottom:2px solid var(--border);flex-shrink:0;height:49px;} 
.card-title{font-family:var(--sans);font-size:20px;font-weight:700;letter-spacing:.8px;text-transform:uppercase;}

.feed-wrap{position:relative;background:#000;display:flex;align-items:center;justify-content:center;overflow:hidden;width:100%;height:100%;flex:1;}
.feed-wrap img {width:100%;height:100%;max-width:100%;max-height:100%;display:block;object-fit:contain;object-position:center;}

.log-card-bottom{height:363px;flex-shrink:0;}
.agv-terminal{background:#030508;font-family:var(--mono);padding:14px;overflow-y:auto;font-size:16px;line-height:1.5;white-space:pre-wrap;font-weight:600;letter-spacing:0.5px;height:303px;flex-shrink:0;}
.parsed-log-item {margin-bottom:2px;border-bottom:1px solid rgba(255,255,255,0.02);padding-bottom:2px;}

.log-cat-manager {color: #00aaff !important;}    
.log-cat-agvmode {color: #E593FF !important;}   
.log-cat-drive {color: #ffbe00 !important;}      
.log-cat-agvstatus {color: #ff3355 !important;}  
.log-cat-default {color: #00e5a0 !important;}    

.side-ctrl-card { flex-shrink: 0; display: flex; flex-direction: column; }

.ctrl-group{padding:6px 14px;display:flex;flex-direction:column;gap:5px;position:relative;width:100%;}
.mode-toggle-container {display:grid;grid-template-columns:1fr 1fr;gap:10px;width:100%;margin-bottom:2px;}
.btn-stop, .btn-follow, .btn-action, .btn-dest, .btn-rediscover, .btn-pickup, .btn-manual, .btn-man-pad {width:100%;font-family:var(--sans);border-radius:8px;cursor:pointer;transition:all 0.15s;}

/* [수정] 각 버튼들의 min-height 속성을 기존 대비 5% 늘려 세로 폭 확장 */
.btn-stop{padding:8px;min-height:46.2px;background:transparent;border:2px solid var(--r);color:var(--r);font-size:21px;font-weight:700}
.btn-stop:hover{background:rgba(255,51,85,0.1);border-color:#fff;box-shadow:0 0 10px rgba(255,51,85,0.3);}
.btn-stop:active{background:var(--r) !important;color:#fff !important;box-shadow:0 0 12px rgba(255,51,85,0.4) !important;}

.btn-follow{padding:8px;min-height:46.2px;background:transparent;border:2px solid #577399;color:#8da9c4;font-size:21px;font-weight:700}
.btn-follow:hover{background:rgba(30,41,59,0.3);border-color:#fff;box-shadow:0 0 10px rgba(30,41,59,0.5);}

.btn-pickup{padding:8px;min-height:46.2px;background:transparent;border:2px solid #ff7700;color:#ff7700;font-size:19px;font-weight:700}
.btn-pickup:hover{background:rgba(255,119,0,0.1);border-color:#fff;box-shadow:0 0 10px rgba(255,119,0,0.3);}

.btn-rediscover{padding:8px;min-height:46.2px;background:transparent;border:2px solid var(--p);color:var(--p);font-size:19px;font-weight:700}
.btn-rediscover:hover{background:rgba(229,147,255,0.1);border-color:#fff;box-shadow:0 0 10px rgba(229,147,255,0.3);}

.btn-manual{padding:8px;min-height:46.2px;background:transparent;border:2px solid var(--g);color:var(--g);font-size:19px;font-weight:700}
.btn-manual:hover{background:rgba(0,229,160,0.1);border-color:#fff;box-shadow:0 0 10px rgba(0,229,160,0.3);}

.btn-action {padding:11px;min-height:52.5px;font-size:19px;font-weight:700;background:transparent;border:2px solid var(--border);color:var(--text);}
.btn-action#btn-guide-main {border-color:var(--b);color:var(--b);}
.btn-action#btn-auto-main {border-color:var(--y);color:var(--y);}
.btn-action#btn-guide-main:hover {background:rgba(0,170,255,0.1);border-color:#fff;}
.btn-auto-main:hover {background:rgba(255,190,0,0.1);border-color:#fff;}

.dest-grid, .manual-grid {
  display: grid;
  gap: 6px;
  width: 100%;
  border: 2px solid transparent;
  border-radius: 10px;
  max-height: 0px;
  opacity: 0;
  padding: 0 2px;
  margin-top: 0px;
  overflow: hidden;
  transition: max-height 0.35s ease, opacity 0.25s ease, padding 0.35s ease, margin 0.35s ease;
  pointer-events: none;
}
.dest-grid { grid-template-columns: repeat(2,1fr); }
.manual-grid { grid-template-columns: repeat(3, 1fr); }

.dest-grid.active-guide-grid { max-height: 240px; opacity: 1; padding: 6px 2px; margin-top: 6px; pointer-events: auto; border-color: rgba(0,170,255,0.2); }
.dest-grid.active-auto-grid { max-height: 240px; opacity: 1; padding: 6px 2px; margin-top: 6px; pointer-events: auto; border-color: rgba(255,190,0,0.2); }
.manual-grid.active-manual-grid { max-height: 150px; opacity: 1; padding: 6px 2px; margin-top: 6px; pointer-events: auto; border-color: rgba(0,229,160,0.2); }

.btn-dest{padding:11px 8px;min-height:50.4px;background:transparent;border:2px solid var(--bhi);color:var(--text);font-size:17px;font-weight:700}
.btn-dest:not(.active-guide):not(.active-auto):hover {border-color:#fff;background:rgba(255,255,255,0.08);}

.btn-man-pad {
  padding: 10px 4px;
  min-height:50.4px;
  background: transparent;
  border: 2px solid var(--bhi);
  color: var(--text);
  font-size: 15px;
  font-weight: 700;
  user-select: none;
  -webkit-user-select: none;
}
.btn-man-pad strong { display: block; font-size: 11px; color: var(--dim); margin-top: 2px; }
.btn-man-pad:hover { border-color: #fff; background: rgba(255,255,255,0.05); }
.btn-man-pad.pressing {
  background: var(--g) !important;
  color: #06080c !important;
  border-color: #fff !important;
  box-shadow: 0 0 10px rgba(0,229,160,0.4);
}
.btn-man-pad.pressing strong { color: #06080c !important; }

#topic-log-card {position: relative; flex: 1; min-height: 140px; display: flex; flex-direction: column;}
.log-list-body {background:#030508;padding:12px;flex: 1;overflow-y:auto;font-family:var(--mono);font-size:16px;display:flex;flex-direction:column;gap:6px;}

.target-select-guide {
  display: none; 
  position: absolute; 
  top: 0; 
  left: 0; 
  width: 100%; 
  height: 100%; 
  z-index: 99; 
  font-size: 20px; 
  font-weight: 800; 
  text-align: center; 
  display: flex;
  align-items: center;
  justify-content: center;
  flex-direction: column;
  gap: 10px;
  background: rgba(3, 5, 8, 0.92) !important; 
  border-radius: 0 0 10px 10px;
  border: none; 
  pointer-events: none;
  animation: pulse-guide 2.0s infinite ease-in-out;
  padding-bottom: 150px;
}
.target-select-guide i { font-size: 26px; }

.guide-theme-blue { color: var(--b) !important; }
.guide-theme-yellow { color: var(--y) !important; }

.active-follow {background:var(--navy) !important;color:#ffffff !important;box-shadow:none !important;border-color:#fff !important;}
.active-guide {background:var(--b) !important;color:#fff !important;box-shadow:none !important;}
.active-auto {background:var(--y) !important;color:#06080c !important;box-shadow:none !important;}
.active-rediscover {background:var(--p) !important;color:#06080c !important;box-shadow:none !important;}
.active-pickup {background:#ff7700 !important;color:#06080c !important;box-shadow:none !important;}
.active-manual {background:var(--g) !important;color:#06080c !important;box-shadow:none !important;}

.log-item {
  display:flex;
  gap:10px;
  border-bottom:1px dashed rgba(27,38,59,0.4);
  padding-bottom:6px;
  font-weight: 600;
  letter-spacing: 0.5px;
}
.log-time {color:#ffffff; font-weight: normal !important;}
.log-msg {color:#ffffff; font-weight: 600;}
.log-clear-btn {background:transparent;border:1px solid var(--r);color:var(--r);padding:3px 8px;font-family:var(--sans);font-size:12px;font-weight:700;border-radius:4px;cursor:pointer;transition:all 0.15s;}
.log-clear-btn:hover {background:var(--r);color:#fff;}
</style>
</head>
<body>

<header>
  <div class="logo-title">AGV Core Dashboard (CycloneDDS)</div>
  <div class="logo-sub">Multi-Stream & Robotics Control Platform</div>
  <div class="badges"><div class="badge live"><div class="dot"></div>SYSTEM ACTIVE</div></div>
</header>

<div class="top-bar">
  <button class="cap-btn" onclick="captureSingle()">📸 스냅샷</button>
  <button class="cap-btn" id="btn-burst" onclick="toggleBurst()">🔁 연속 캡처</button>
  <button class="cap-btn" id="btn-video" onclick="toggleVideo()">🎥 동영상</button>
</div>

<div class="main">
  <div class="center-panel">
    <div class="video-grid">
      <div class="card">
        <div class="card-hd"><span class="card-title" style="color:var(--g)">전방 화면 ({{ stream_1 }})</span></div>
        <div class="feed-wrap"><img src="{{ stream_1 }}" alt="Cam 1"></div>
      </div>
      <div class="card">
        <div class="card-hd"><span class="card-title" style="color:var(--p)">지도 화면 ({{ stream_2 }})</span></div>
        <div class="feed-wrap"><img src="{{ stream_2 }}" alt="Cam 2"></div>
      </div>
    </div>
    <div class="card log-card-bottom">
      <div class="card-hd">
        <span class="card-title" style="color:var(--y)">AGV 로그 (/AGV_log)</span>
        <button class="log-clear-btn" onclick="clearAgvLog()">CLEAR</button>
      </div>
      <div class="agv-terminal" id="agv-console">대기 중... 외부 로봇 통합 이벤트 로그를 실시간 대기하고 있습니다.</div>
    </div>
  </div>

  <div class="side-panel">
    <div class="card side-ctrl-card">
      <div class="card-hd"><span class="card-title" style="color:#fff;">AGV REMOTE CONTROL</span></div>
      
      <div class="ctrl-group" style="border-bottom:2px solid var(--border); padding-bottom:10px;">
        <button id="btn-stop-core" class="btn-stop" onclick="clickStop()">정지</button>
      </div>
      
      <div class="ctrl-group" style="border-bottom:2px solid var(--border); padding-top:10px; padding-bottom:10px;">
        <button id="btn-follow-core" class="btn-follow" onclick="clickFollow()">추종</button>
      </div>
      
      <div class="ctrl-group" style="border-bottom:2px solid var(--border); padding-top:10px; padding-bottom:10px;">
        <div class="mode-toggle-container">
          <button id="btn-guide-main" class="btn-action" onclick="toggleMode('guide')">안내</button>
          <button id="btn-auto-main" class="btn-action" onclick="toggleMode('auto')">자율주행</button>
        </div>
        <div id="dest-group-grid" class="dest-grid">
          <button id="dest-0" class="btn-dest" onclick="clickDest(0)">목적지 0</button>
          <button id="dest-1" class="btn-dest" onclick="clickDest(1)">목적지 1</button>
          <button id="dest-2" class="btn-dest" onclick="clickDest(2)">목적지 2</button>
          <button id="dest-3" class="btn-dest" onclick="clickDest(3)">목적지 3</button>
          <button id="dest-4" class="btn-dest" onclick="clickDest(4)">목적지 4</button>
          <button id="dest-5" class="btn-dest" onclick="clickDest(5)">목적지 5</button>
          <button id="dest-6" class="btn-dest" onclick="clickDest(6)">목적지 6</button>
          <button id="dest-7" class="btn-dest" onclick="clickDest(7)">목적지 7</button>
        </div>
      </div>
      
      <div class="ctrl-group" style="border-bottom:2px solid var(--border); padding-top:10px; padding-bottom:10px;">
        <button id="btn-pickup-core" class="btn-pickup" onclick="clickPickup()">픽업</button>
      </div>
      
      <div class="ctrl-group" style="border-bottom:2px solid var(--border); padding-top:10px; padding-bottom:10px;">
        <button id="btn-rediscover-core" class="btn-rediscover" onclick="clickRediscover()">현 위치 재탐색</button>
      </div>

      <div class="ctrl-group" style="padding-top:10px; padding-bottom:12px;">
        <button id="btn-manual-core" class="btn-manual" onclick="toggleManualMode()">수동 조작</button>
        
        <div id="manual-control-grid" class="manual-grid">
          <button id="p-ccw" class="btn-man-pad" data-cmd="ccw">좌회전<br><strong>(Q)</strong></button>
          <button id="p-forward" class="btn-man-pad" data-cmd="forward">전진<br><strong>(W / ↑)</strong></button>
          <button id="p-cw" class="btn-man-pad" data-cmd="cw">우회전<br><strong>(E)</strong></button>
          <button id="p-left" class="btn-man-pad" data-cmd="left">좌<br><strong>(A / ←)</strong></button>
          <button id="p-backward" class="btn-man-pad" data-cmd="backward">후진<br><strong>(S / ↓)</strong></button>
          <button id="p-right" class="btn-man-pad" data-cmd="right">우<br><strong>(D / →)</strong></button>
        </div>
      </div>
    </div>
    
    <div class="card" id="topic-log-card">
      <div class="card-hd" id="topic-log-header">
        <span class="card-title" style="color:var(--b)">발행 토픽 로그</span>
        <button class="log-clear-btn" onclick="clearLog()">CLEAR</button>
      </div>
      <div class="log-list-body" id="log-container">
        <div style="color:var(--dim); font-style:italic;">발행된 원격 명령 토픽 내역이 없습니다.</div>
      </div>
      
      <div id="text-dest-guide" class="target-select-guide">
        <i class="fa-solid fa-hand-point-up"></i>
        <span>목적지를 선택해 주세요!</span>
      </div>
    </div>
  </div>
</div>

<script>
let currentActiveMode = null; 
let previousLogData = ""; 
let previousPubLogLength = -1;
let lastUiStateStr = "";

let manualIntervalId = null;
let activeManualCmd = null;

const keyMap = { 
  'q': 'ccw', 
  'w': 'forward', 'arrowup': 'forward',
  'e': 'cw', 
  'a': 'left', 'arrowleft': 'left',
  's': 'backward', 'arrowdown': 'backward',
  'd': 'right', 'arrowright': 'right'
};

const reverseKeyMap = { 'ccw': 'p-ccw', 'forward': 'p-forward', 'cw': 'p-cw', 'left': 'p-left', 'backward': 'p-backward', 'right': 'p-right' };

function applyUiState(state) {
  const guideBtn = document.getElementById('btn-guide-main');
  const autoBtn = document.getElementById('btn-auto-main');
  const manualBtn = document.getElementById('btn-manual-core');
  const guideText = document.getElementById('text-dest-guide');
  const destGrid = document.getElementById('dest-group-grid');
  const manGrid = document.getElementById('manual-control-grid');
  
  if (!state.activeMode && !state.activeType) {
    currentActiveMode = null;
    guideText.style.display = 'none';
    destGrid.classList.remove('active-guide-grid', 'active-auto-grid');
    manGrid.classList.remove('active-manual-grid');
    clearAllActiveStyles();
    stopManualPublish();
    return;
  }

  if (state.activeType) {
    currentActiveMode = null;
    guideText.style.display = 'none';
    destGrid.classList.remove('active-guide-grid', 'active-auto-grid');
    clearAllActiveStyles();
    
    if (state.activeType !== 'manual') {
      manGrid.classList.remove('active-manual-grid');
      stopManualPublish();
    }

    if (state.activeType === 'follow') {
      document.getElementById('btn-follow-core').classList.add('active-follow');
    } else if (state.activeType === 'rediscover') {
      document.getElementById('btn-rediscover-core').classList.add('active-rediscover');
    } else if (state.activeType === 'pickup') {
      document.getElementById('btn-pickup-core').classList.add('active-pickup');
    } else if (state.activeType === 'manual') {
      manualBtn.classList.add('active-manual');
      manGrid.classList.add('active-manual-grid');
    } else if (state.activeType === 'stop') {
      document.getElementById('btn-stop-core').classList.add('active-stop'); // 필요시 스타일 연동
    }
    return;
  }

  if (state.activeMode) {
    currentActiveMode = state.activeMode;
    clearAllActiveStyles();
    manGrid.classList.remove('active-manual-grid');
    stopManualPublish();
    
    guideText.classList.remove('guide-theme-blue', 'guide-theme-yellow');

    if (state.activeMode === 'guide') {
      guideBtn.classList.add('active-guide');
      destGrid.classList.remove('active-auto-grid');
      destGrid.classList.add('active-guide-grid');
      guideText.style.display = 'none'; 
      if (state.activeDest !== null) {
        const targetDestBtn = document.getElementById('dest-' + state.activeDest);
        if(targetDestBtn) targetDestBtn.classList.add('active-guide');
      }
    } else if (state.activeMode === 'auto') {
      autoBtn.classList.add('active-auto');
      destGrid.classList.remove('active-guide-grid');
      destGrid.classList.add('active-auto-grid');
      guideText.style.display = 'none';
      if (state.activeDest !== null) {
        const targetDestBtn = document.getElementById('dest-' + state.activeDest);
        if(targetDestBtn) targetDestBtn.classList.add('active-auto');
      }
    }
  }
}

function toggleMode(mode) {
  fetch('/api/ui/set_mode', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({ mode: mode })
  });
}

function toggleManualMode() {
  fetch('/api/ui/set_mode', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({ mode: 'manual_trigger' }) 
  });
}

function clickDest(num) {
  if (!currentActiveMode) return;
  let finalTopicValue = (currentActiveMode === 'guide') ? 20 + num : 30 + num;
  let logLabel = `${(currentActiveMode === 'guide' ? '안내' : '자율주행')} - 목적지 ${num}`;
  pubCmd(finalTopicValue, logLabel);
}

function clickFollow() { pubCmd(10, '추종'); }
function clickRediscover() { pubCmd(40, '현 위치 재탐색'); }
function clickPickup() { pubCmd(50, '픽업'); }
function clickStop() { 
  stopManualPublish();
  pubCmd(0, '정지'); 
}

function clearAllActiveStyles() {
  const allUiButtons = document.querySelectorAll('button:not(.cap-btn):not(.log-clear-btn):not(.btn-man-pad)');
  allUiButtons.forEach(btn => btn.classList.remove('active-follow', 'active-guide', 'active-auto', 'active-rediscover', 'active-pickup', 'active-manual', 'active-stop'));
  const destButtons = document.querySelectorAll('.btn-dest');
  destButtons.forEach(btn => btn.classList.remove('active-guide', 'active-auto'));
}

function startManualPublish(cmd) {
  if (activeManualCmd === cmd) return;
  stopManualPublish();
  activeManualCmd = cmd;
  
  const btnId = reverseKeyMap[cmd];
  if(btnId) document.getElementById(btnId).classList.add('pressing');

  sendManualTick(cmd);
  manualIntervalId = setInterval(() => {
    sendManualTick(cmd);
  }, 100);
}

function stopManualPublish() {
  if (manualIntervalId) {
    clearInterval(manualIntervalId);
    manualIntervalId = null;
  }
  if (activeManualCmd) {
    const btnId = reverseKeyMap[activeManualCmd];
    if(btnId) document.getElementById(btnId).classList.remove('pressing');
    sendManualStop(); 
    activeManualCmd = null;
  }
}

async function sendManualTick(cmd) {
  try {
    await fetch('/api/pub/manual_cmd', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ command: cmd })
    });
  } catch(e) {}
}

async function sendManualStop() {
  try {
    await fetch('/api/pub/manual_cmd', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ command: 'stop' })
    });
  } catch(e) {}
}

document.querySelectorAll('.btn-man-pad').forEach(btn => {
  const cmd = btn.getAttribute('data-cmd');
  btn.addEventListener('mousedown', (e) => { e.preventDefault(); startManualPublish(cmd); });
  btn.addEventListener('touchstart', (e) => { e.preventDefault(); startManualPublish(cmd); });
});

window.addEventListener('mouseup', stopManualPublish);
window.addEventListener('touchend', stopManualPublish);

window.addEventListener('keydown', (e) => {
  const key = e.key.toLowerCase();
  if (keyMap[key]) {
    e.preventDefault();
    startManualPublish(keyMap[key]);
  }
});
window.addEventListener('keyup', (e) => {
  const key = e.key.toLowerCase();
  if (keyMap[key] && activeManualCmd === keyMap[key]) {
    stopManualPublish();
  }
});

function renderPubLogs(logList) {
  const container = document.getElementById('log-container');
  if (logList.length === 0) {
    container.innerHTML = '<div style="color:var(--dim); font-style:italic;">발행된 원격 명령 토픽 내역이 없습니다.</div>';
    return;
  }
  
  let html = "";
  for (let i = 0; i < logList.length; i++) {
    let item = logList[i];
    html += `<div class="log-item"><span class="log-time">${item.time}</span><span class="log-msg" style="color:#ffffff">→ <span style="color:var(--y)">${item.val}</span> <span style="color:#e2e8f0; font-size:14px;">(${item.label})</span></span></div>`;
  }
  container.innerHTML = html;
  container.scrollTop = container.scrollHeight;
}

async function clearLog() { try { await fetch('/api/clear/pub_log', { method: 'POST' }); } catch(e){} }
async function clearAgvLog() {
  try {
    const res = await fetch('/api/clear/agv_log', { method: 'POST' });
    const d = await res.json();
    if (d.status === 'success') {
      previousLogData = "";
      document.getElementById('agv-console').innerHTML = "대기 중... 외부 로봇 통합 이벤트 로그를 실시간 대기하고 있습니다.";
    }
  } catch(e) {}
}

async function pubCmd(val, label) {
  try {
    await fetch('/api/pub/nano_status', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({command: parseInt(val, 10), label: label})
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

function renderParsedLog(rawText) {
  if (!rawText) return "대기 중... 외부 로봇 통합 이벤트 로그를 실시간 대기하고 있습니다.";
  const lines = rawText.split('\n');
  
  let resultHtml = "";
  lines.forEach(line => {
    if (!line.trim()) return;
    let colorClass = "log-cat-default";
    
    if (line.includes("[매니저]")) colorClass = "log-cat-manager";
    else if (line.includes("[agv_mode]")) colorClass = "log-cat-agvmode";
    else if (line.includes("[주행]")) colorClass = "log-cat-drive";
    else if (line.includes("[AGV_status]")) colorClass = "log-cat-agvstatus";
    
    resultHtml += `<div class="parsed-log-item ${colorClass}">${line}</div>`;
  });
  return resultHtml || "대기 중... 외부 로봇 통합 이벤트 로그를 실시간 대기하고 있습니다.";
}

async function updateLoop() {
  try {
    const d = await (await fetch('/api/status')).json();
    
    const con = document.getElementById('agv-console');
    if (d.agv_log !== previousLogData) {
      const isScrolledToBottom = (con.scrollHeight - con.scrollTop - con.clientHeight) <= 1;
      previousLogData = d.agv_log;
      con.innerHTML = renderParsedLog(d.agv_log);
      if (isScrolledToBottom) { con.scrollTop = con.scrollHeight; }
    }
    
    if (d.pub_logs && d.pub_logs.length !== previousPubLogLength) {
      previousPubLogLength = d.pub_logs.length;
      renderPubLogs(d.pub_logs);
    }
    
    const currentUiStateStr = JSON.stringify(d.ui_state);
    if (currentUiStateStr !== lastUiStateStr) {
      lastUiStateStr = currentUiStateStr;
      applyUiState(d.ui_state);
    }
  } catch(e){}
  setTimeout(updateLoop, 200);
}

async function toggleVideo() {
  try {
    const res = await fetch('/api/video/toggle', {method:'POST'});
    const d = await res.json();
    document.getElementById('btn-video').classList.toggle('bursting', d.recording);
  } catch(e){}
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
        self._agv_log_buffer = deque(maxlen=100)
        self._agv_log_buffer.append("[시스템] 🟢 CycloneDDS 대시보드 코어 로깅 허브 노드가 활성화되었습니다.")
        self._pub_log_buffer = [] 
        
        self._current_ui_state = {
            "activeMode": None,
            "activeDest": None,
            "activeType": None
        }
        
        self._bursting = False
        self._recording = False
        self._ui_reset_request = False
        self._capture_seq = 0

        os.makedirs(CAPTURE_DIR, exist_ok=True)
        os.makedirs(MOVIE_DIR, exist_ok=True)

        self.create_subscription(String, 'traffic_sign_topic', self._cb_sign, 10)
        self.create_subscription(String, '/control_state', self._cb_brain, 10)
        self.create_subscription(String, '/AGV_log', self._cb_agv_log, 10)
        
        self.create_subscription(Int32, '/agv_mode', self._cb_agv_mode_sub, 10)
        self.create_subscription(Int32, '/AGV_status', self._cb_agv_status, 10)
        
        # [추가] /ARM_status 구독 설정
        self.create_subscription(Int32, '/ARM_status', self._cb_arm_status, 10)
        
        self._pub_nano = self.create_publisher(Int32, 'nano_send_status', 10)
        self._pub_manual = self.create_publisher(String, 'manual_cmd', 10)

    def _cb_sign(self, msg): pass
    def _cb_brain(self, msg): pass
    
    def _cb_agv_log(self, msg):
        with self._lock: self._agv_log_buffer.append(f"[{datetime.now().strftime('%H:%M:%S')}] {msg.data}")

    def _cb_agv_mode_sub(self, msg):
        with self._lock:
            self._agv_log_buffer.append(f"[{datetime.now().strftime('%H:%M:%S')}] [agv_mode] -> {msg.data}")

    def clear_agv_log_buffer(self):
        with self._lock: self._agv_log_buffer.clear()
        
    def clear_pub_log_buffer(self):
        with self._lock: self._pub_log_buffer.clear()

    def _cb_agv_status(self, msg):
        with self._lock:
            # [수정] 현재 상태가 'pickup'(픽업모드)인 경우 수신된 AGV_status 값을 무시하고 리턴합니다.
            if self._current_ui_state.get("activeType") == "pickup":
                return
                
            self._agv_log_buffer.append(f"[{datetime.now().strftime('%H:%M:%S')}] [AGV_status] -> {msg.data}")

        if msg.data in [0, 1]:
            with self._lock: 
                self._ui_reset_request = True
                self._current_ui_state = {"activeMode": None, "activeDest": None, "activeType": "stop" if msg.data == 0 else None}

    # [추가] /ARM_status 메시지 콜백 함수
    def _cb_arm_status(self, msg):
        if msg.data == 1:
            with self._lock:
                self._agv_log_buffer.append(f"[{datetime.now().strftime('%H:%M:%S')}] [ARM_status] -> 1 (정지 명령 실행)")
            # 정지 명령 토픽(0) 발행 및 UI 상태를 stop으로 변경
            self.publish_nano_command(0, "ARM_status에 의한 정지")

    def set_ui_mode(self, mode):
        with self._lock:
            if mode == 'manual_trigger':
                self._current_ui_state = {"activeMode": None, "activeDest": None, "activeType": "manual"}
                ts = datetime.now().strftime("[%H:%M:%S]")
                self._pub_log_buffer.append({"time": ts, "val": 60, "label": "수동 조작 모드"})
                
                msg = Int32()
                msg.data = 60
                self._pub_nano.publish(msg)
            else:
                self._current_ui_state["activeMode"] = mode
                self._current_ui_state["activeDest"] = None
                self._current_ui_state["activeType"] = None

    def update_ui_state_by_cmd(self, cmd_int, label):
        with self._lock:
            if cmd_int == 0:
                self._current_ui_state = {"activeMode": None, "activeDest": None, "activeType": "stop"}
            elif cmd_int == 10:
                self._current_ui_state = {"activeMode": None, "activeDest": None, "activeType": "follow"}
            elif cmd_int == 40:
                self._current_ui_state = {"activeMode": None, "activeDest": None, "activeType": "rediscover"}
            elif cmd_int == 50:
                self._current_ui_state = {"activeMode": None, "activeDest": None, "activeType": "pickup"}
            elif cmd_int == 60:
                self._current_ui_state = {"activeMode": None, "activeDest": None, "activeType": "manual"}
            elif 20 <= cmd_int <= 27:
                self._current_ui_state["activeMode"] = "guide"
                self._current_ui_state["activeDest"] = cmd_int - 20
                self._current_ui_state["activeType"] = None
            elif 30 <= cmd_int <= 37:
                self._current_ui_state["activeMode"] = "auto"
                self._current_ui_state["activeDest"] = cmd_int - 30
                self._current_ui_state["activeType"] = None
            
            ts = datetime.now().strftime("[%H:%M:%S]")
            self._pub_log_buffer.append({"time": ts, "val": cmd_int, "label": label})

    def publish_nano_command(self, cmd_int, label):
        self.update_ui_state_by_cmd(cmd_int, label)
        msg = Int32()
        msg.data = cmd_int
        self._pub_nano.publish(msg)

    def publish_manual_command(self, cmd_str):
        msg = String()
        msg.data = cmd_str
        self._pub_manual.publish(msg)
        
        if cmd_str != "stop":
            with self._lock:
                ts = datetime.now().strftime("[%H:%M:%S]")
                if not self._pub_log_buffer or self._pub_log_buffer[-1]["label"] != f"수동 조작: {cmd_str}":
                    self._pub_log_buffer.append({"time": ts, "val": "CMD", "label": f"수동 조작: {cmd_str}"})

    def capture_single(self):
        threading.Thread(target=self._save_snapshot, daemon=True).start()
        return True

    def _save_snapshot(self):
        try:
            cap = cv2.VideoCapture(STREAM_URL_1)
            ret, frame = cap.read()
            cap.release()
            if ret:
                with self._lock: self._capture_seq += 1
                ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                cv2.imwrite(os.path.join(CAPTURE_DIR, f"snapshot_{ts}.jpg"), frame)
        except: pass

    def toggle_burst(self):
        self._bursting = not self._bursting
        if self._bursting: threading.Thread(target=self._burst_loop, daemon=True).start()
        return self._bursting

    def _burst_loop(self):
        while self._bursting:
            self._save_snapshot()
            time.sleep(BURST_INTERVAL)

    def toggle_video_record(self):
        self._recording = not self._recording
        if self._recording: threading.Thread(target=self._record_video, daemon=True).start()
        return self._recording

    def _record_video(self):
        cap = cv2.VideoCapture(STREAM_URL_1)
        out = cv2.VideoWriter(os.path.join(MOVIE_DIR, f"video_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"), cv2.VideoWriter_fourcc(*'mp4v'), 20.0, (640, 480))
        while self._recording:
            ret, frame = cap.read()
            if ret: out.write(frame)
        out.release()
        cap.release()

    def get_status(self):
        with self._lock:
            req = self._ui_reset_request
            self._ui_reset_request = False
            return {
                "agv_log": "\n".join(self._agv_log_buffer), 
                "pub_logs": list(self._pub_log_buffer),
                "ui_state": dict(self._current_ui_state),
                "bursting": self._bursting, 
                "ui_reset_request": req
            }

def create_app(node):
    app = Flask(__name__)
    @app.route('/')
    def index(): return render_template_string(HTML, stream_1=STREAM_URL_1, stream_2=STREAM_URL_2)
    @app.route('/api/status')
    def api_status(): return jsonify(node.get_status())
    @app.route('/api/pub/nano_status', methods=['POST'])
    def api_pub_nano():
        req_data = request.get_json()
        cmd = req_data.get('command', 0)
        label = req_data.get('label', '')
        node.publish_nano_command(cmd, label)
        return jsonify({"status": "success"})
    @app.route('/api/pub/manual_cmd', methods=['POST'])
    def api_pub_manual():
        req_data = request.get_json()
        cmd_str = req_data.get('command', 'stop')
        node.publish_manual_command(cmd_str)
        return jsonify({"status": "success"})
    @app.route('/api/ui/set_mode', methods=['POST'])
    def api_ui_set_mode():
        mode = request.get_json().get('mode', None)
        node.set_ui_mode(mode)
        return jsonify({"status": "success"})
    @app.route('/api/clear/agv_log', methods=['POST'])
    def api_clear_agv_log():
        node.clear_agv_log_buffer()
        return jsonify({"status": "success"})
    @app.route('/api/clear/pub_log', methods=['POST'])
    def api_clear_pub_log():
        node.clear_pub_log_buffer()
        return jsonify({"status": "success"})
    @app.route('/api/capture/single', methods=['POST'])
    def api_capture_single(): return jsonify({"result": node.capture_single()})
    @app.route('/api/video/toggle', methods=['POST'])
    def api_video_toggle(): return jsonify({"recording": node.toggle_video_record()})
    @app.route('/api/burst/toggle', methods=['POST'])
    def api_burst_toggle(): return jsonify({"bursting": node.toggle_burst()})
    return app

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    threading.Thread(target=lambda: create_app(node).run(host='0.0.0.0', port=5000, threaded=True), daemon=True).start()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__':
    main()