import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    base_dir = os.path.abspath(os.path.dirname(__file__))

    # 수동 조작 전용 파이썬 노드 실행 (빠른 로그 출력을 위해 -u 옵션 추가)
    manual_node = ExecuteProcess(
        cmd=['python3', '-u', os.path.join(base_dir, 'manual_control.py')],
        output='screen'
    )

    return LaunchDescription([
        manual_node
    ])