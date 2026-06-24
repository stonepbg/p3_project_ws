import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # 1. 하드웨어 및 오도메트리 실행
    odometry_process = ExecuteProcess(
        cmd=['ros2', 'launch', 'myagv_odometry', 'myagv_active.launch.py'],
        output='screen'
    )

    # 2. 내비게이션 및 맵 실행
    navigation_process = ExecuteProcess(
        cmd=['ros2', 'launch', 'myagv_navigation2', 'navigation2_active.launch.py', 'map:=/home/er/my_map.yaml'],
        output='screen'
    )

    # 3. [신규] 초기 위치 파악(비전) 실행
    vision_process = ExecuteProcess(
        cmd=['ros2', 'launch', 'vision_localization.launch.py'],
        output='screen'
    )

    # ==========================================
    # 감시자(Watcher) 노드 설정
    # ==========================================
    # 감시자 1: 라이다 센서 데이터(/scan)가 1회 발행될 때까지 숨어서 대기
    wait_for_hardware = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '--once', '/scan'],
        output='log'
    )

    # 감시자 2: 맵 데이터(/map)가 1회 발행될 때까지 숨어서 대기
    wait_for_navigation = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '--once', '/map'],
        output='log'
    )

    # ==========================================
    # 이벤트 핸들러(트리거) 설정
    # ==========================================
    # 트리거 1: 하드웨어 감시자가 종료되면(데이터 확인됨) 내비게이션 실행
    trigger_navigation = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_hardware,
            on_exit=[
                LogInfo(msg="✅ [시스템] 하드웨어 가동 확인 완료. 내비게이션(RViz)을 시작합니다."),
                navigation_process
            ]
        )
    )

    # 트리거 2: 내비게이션 감시자가 종료되면(맵 로딩 확인됨) 비전 위치 파악 실행
    trigger_vision = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_navigation,
            on_exit=[
                LogInfo(msg="✅ [시스템] 맵 로딩 확인 완료. 초기 위치 파악을 시작합니다."),
                vision_process
            ]
        )
    )

    return LaunchDescription([
        # 1. 가장 먼저 오도메트리와 감시자들을 백그라운드에 깔아둡니다.
        odometry_process,
        wait_for_hardware,
        wait_for_navigation,
        
        # 2. 감시자가 신호를 보내면 작동할 트리거들을 등록합니다.
        trigger_navigation,
        trigger_vision
    ])