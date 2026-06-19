import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    base_dir = os.path.abspath(os.path.dirname(__file__))

    follower_node = ExecuteProcess(
        # python3 바로 뒤에 '-u' (unbuffered) 옵션을 추가합니다.
        cmd=['python3', '-u', os.path.join(base_dir, 'person_follower.py')],
        output='screen'
    )

    return LaunchDescription([
        follower_node
    ])