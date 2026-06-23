import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    base_dir = os.path.abspath(os.path.dirname(__file__))

    # 위치 파악 대기 없이 즉시 자율주행 커맨더 노드 실행 (안내 및 이동 모드 공통)
    nav2_commander_node = ExecuteProcess(
        cmd=['python3', os.path.join(base_dir, 'nav2_commander.py')],
        output='screen'
    )

    return LaunchDescription([
        nav2_commander_node
    ])