import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    base_dir = os.path.abspath(os.path.dirname(__file__))

    # 정밀 밀착 전용 파이썬 노드 실행 (디버깅을 위해 -u 옵션 추가)
    aligner_node = ExecuteProcess(
        cmd=['python3', '-u', os.path.join(base_dir, 'pickup_aligner.py')],
        output='screen'
    )

    return LaunchDescription([
        aligner_node
    ])