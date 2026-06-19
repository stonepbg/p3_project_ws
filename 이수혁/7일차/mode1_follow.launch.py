import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    base_dir = os.path.abspath(os.path.dirname(__file__))

    # 사람 추종 노드 단독 실행 (카메라 및 비전 노드 제외)
    follower_node = ExecuteProcess(
        cmd=['python3', os.path.join(base_dir, 'person_follower.py')],
        output='screen'
    )

    return LaunchDescription([
        follower_node
    ])