import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, EmitEvent, RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    base_dir = os.path.abspath(os.path.dirname(__file__))

    # 1. 카메라 영상 송출 노드
    camera_node = ExecuteProcess(
        cmd=['python3', os.path.join(base_dir, 'csi_camera_pub.py')],
        output='screen'
    )

    # 2. 마커 인식 및 위치 파악 노드
    localization_node = ExecuteProcess(
        cmd=['python3', os.path.join(base_dir, 'aruco_localization.py')],
        output='screen'
    )

    # 3. 자동 종료 이벤트 핸들러: 위치 파악 노드가 끝나면 전체 Launch를 강제 종료함
    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=localization_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return LaunchDescription([
        camera_node,
        localization_node,
        exit_event_handler
    ])
