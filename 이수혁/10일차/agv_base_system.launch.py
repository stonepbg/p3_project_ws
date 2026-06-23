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

    # 3. [신규] 초기 위치 파악(비전) 실행
    vision_process = ExecuteProcess(
        cmd=['ros2', 'launch', 'vision_localization.launch.py'],
        output='screen'
    )

    # 하드웨어 가동 후 5초 대기 -> 내비게이션 맵 켜기
    delayed_navigation = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="[시스템] 하드웨어 가동 완료. 5초 대기 끝! 지정된 맵으로 내비게이션을 시작합니다."),
            navigation_process
        ]
    )

    # 내비게이션 가동 후 5초 대기 (총 10초) -> 초기 위치 파악 실행
    delayed_vision = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="[시스템] 내비게이션 가동 완료. 초기 위치 파악(AMCL 캘리브레이션)을 자동 시작합니다."),
            vision_process
        ]
    )

    return LaunchDescription([
        odometry_process,
        delayed_navigation,
        delayed_vision
    ])