# ROS2 토픽 명세 (follow_ws)

> Jetson Orin Nano 측 노드 간 통신 토픽 정리
> 도메인 격리: Orin 노드 = domain 5, 외부(대시보드/AGV/팔) = domain 0
> `domain_bridge`가 선택 토픽을 0↔5 중계

---

## 모드 명령 계열

| 토픽 | 방향 | 타입 | 값 | 의미/역할 |
|---|---|---|---|---|
| `/nano_send_status` | 대시보드→fsm (0→5) | Int32 | 0 | 정지모드 |
| | | | 10~17 | 추종모드 (끝자리=목적지) |
| | | | 20~27 | 안내모드 (끝자리=목적지) |
| | | | 30~37 | 자율주행모드 (끝자리=목적지) |
| | | | 40~47 | 위치재조정모드 |
| | | | 50 | 픽업·유도 |
| | | | 51 | 픽업·좌표전송 |
| | | | 60 | 수동모드 (manual) |
| `/agv_mode` | fsm→AGV (5→0) | Int32 | 위와 동일 | fsm이 받은 모드를 AGV로 바이패스. ack 올 때까지 0.5초마다 최대 10회 재발행 |
| `/AGV_mode_ack` | AGV→fsm (0→5) | Int32 | =agv_mode 값 | 해당 모드 수신 확인 (핸드셰이크) |
| | | | 51 | 픽업 좌표전송 진입 → pickup_target 송출 시작 |
| | | | 52 | 픽업 좌표 락온 → pickup_target 송출 중단 |

## AGV 상태 계열

| 토픽 | 방향 | 타입 | 값 | 의미 (모드에 따라 해석) |
|---|---|---|---|---|
| `/AGV_status` | AGV→fsm (0→5) | Int32 | 0 | 주행시작 / 안내시작(후방판단 ON) / 로컬라이제이션 완료 |
| | | | 1 | 도착·완료 / 밀착완료(픽업) |
| | | | 2 | 장애물 감지 (회피 멘트) |
| | | | 4 | 재도킹 좌표요청 (pickup_target 재송출) |

## 추종 계열

| 토픽 | 방향 | 타입 | 값 | 역할 |
|---|---|---|---|---|
| `/follow_enable` | fsm→follow | Bool | True/False | 추종 카테고리(10번대)만 True |
| `/target_cmd` | follow/pickup→AGV (5→0) | Twist | linear.z=모드 | 0=추종, 1=좌탐색, 2=우탐색, 3=find(정지) |
| | | | linear.x=거리(m) | 타겟까지 거리 |
| | | | angular.z=각도(rad) | 좌우 조향 |
| `/rear_person` | follow→fsm | Bool | True/False | 안내모드 후방 사람 유무 |
| `/guide_pause` | fsm→AGV (5→0) | Bool | True/False | 안내모드 대기(True)/주행(False) |
| `/cam_health` | follow→monitor | Float32 | fps 값 | 카메라 루프 fps |

## 픽업·블록인식 계열

| 토픽 | 방향 | 타입 | 값 | 역할 |
|---|---|---|---|---|
| `/laser_point` | follow→pickup | Point | x,y,z(m) | 레이저 지점 3D 좌표 (카메라 광학) |
| `/selected_block` | follow→pickup | Point | x,y,z(m) | 선택된 블록 3D 좌표 (카메라 광학) |
| `/selected_block_class` | follow→pickup | String | 클래스명 | 블록 종류 (obj_3 등) |
| `/pickup_active` | fsm→pickup | Bool | True/False | 좌표 송출 게이트 (51 ack 시 True, 52/완료 시 False) |
| `/pickup_target` | pickup→AGV (5→0) | Point | x,y,z(m) | AGV base_link 기준 도킹 좌표 |
| `/object_pose` | pickup→팔 (5→0) | PoseStamped | x,y,z(m) | 팔 파지 좌표 (재탐색 시 3회 발행) |

## 로봇팔 계열

| 토픽 | 방향 | 타입 | 값 | 역할 |
|---|---|---|---|---|
| `/ARM_status` | 팔→fsm/follow/pickup (0→5) | Int32 | 0 | 재파지 시작 (REGRIP) |
| | | | 1 | 팔 작업 완료 → "픽업 완료!" + 픽업모드 종료 (한 번만 발행) |

## TTS·모니터 계열

| 토픽 | 방향 | 타입 | 값 | 역할 |
|---|---|---|---|---|
| `/tts_text` | fsm→tts | String | 멘트 문자열 | 음성 재생 (큐 순차 처리) |
| `/pickup_status` | pickup→follow/monitor | String | STABLE/PUB/MISMATCH | 픽업 좌표 안정성 상태 |
| | | | DONE | 재탐색 완료 → arm_phase 해제 |
| | | | REGRIP | 재파지 시작 → arm_phase 재개 |
| `/heartbeat/<node>` | 각 노드→monitor | String | JSON | 노드별 생존·상태 (follow/fsm/tts/pickup) |
