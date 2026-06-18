import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    base_dir = os.path.abspath(os.path.dirname(__file__))

    # [1단계] 기존에 만들어둔 위치 파악 런치 파일을 프로세스로 직접 실행
    localization_launch_process = ExecuteProcess(
        cmd=['ros2', 'launch', os.path.join(base_dir, 'vision_localization.launch.py')],
        output='screen'
    )

    # [2단계] 위치 파악 완료 후 실행될 주행 커맨더 노드
    nav2_commander_node = ExecuteProcess(
        cmd=['python3', os.path.join(base_dir, 'nav2_commander.py')],
        output='screen'
    )

    # 이벤트 자동화: 위치 파악 런치 프로세스가 완전히 종료되면(위치 동기화 후 자동 종료됨) 주행 커맨더 시작
    start_guide_sequence = RegisterEventHandler(
        OnProcessExit(
            target_action=localization_launch_process,
            on_exit=[
                LogInfo(msg="✅ [안내 모드] vision_localization 런치가 종료되었습니다. 자율주행 커맨더를 시작합니다."),
                nav2_commander_node
            ]
        )
    )

    return LaunchDescription([
        localization_launch_process,
        start_guide_sequence
    ])