import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo

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

    # 3. 초기 위치 파악(비전) 실행
    vision_process = ExecuteProcess(
        cmd=['ros2', 'launch', 'vision_localization.launch.py'],
        output='screen'
    )

    # ==========================================
    # FastRTPS 환경을 위한 안전한 시간 대기(TimerAction)
    # ==========================================
    
    # 하드웨어가 켜지고 8초 동안 충분히 기다린 후 내비게이션(RViz) 실행
    delayed_navigation = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="✅ [시스템] 8초 대기 완료. 내비게이션(RViz)을 시작합니다."),
            navigation_process
        ]
    )

    # 전체 시스템 가동 시작 후 20초(내비 켜지고 12초 후) 뒤에 비전 위치 탐색 실행
    # (RViz와 맵이 완전히 로딩될 수 있도록 넉넉하게 시간을 줍니다)
    delayed_vision = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg="✅ [시스템] 맵 로딩 대기 완료. 초기 위치 파악을 시작합니다."),
            vision_process
        ]
    )

    return LaunchDescription([
        odometry_process,
        delayed_navigation,
        delayed_vision
    ])
