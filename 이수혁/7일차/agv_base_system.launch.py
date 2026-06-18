import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. myagv_odometry 패키지의 myagv_active.launch.py 포함
    odometry_pkg_dir = get_package_share_directory('myagv_odometry')
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odometry_pkg_dir, 'launch', 'myagv_active.launch.py')
        )
    )

    # 2. myagv_navigation2 패키지의 navigation2_active.launch.py 포함 및 맵 경로 매개변수 전달
    navigation_pkg_dir = get_package_share_directory('myagv_navigation2')
    map_file_path = '/home/er/my_map.yaml'  # 지정해주신 맵 파일 경로
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_pkg_dir, 'launch', 'navigation2_active.launch.py')
        ),
        launch_arguments={'map': map_file_path}.items()
    )

    # 두 개의 런치 파일을 동시에 실행하도록 묶어서 반환
    return LaunchDescription([
        odometry_launch,
        navigation_launch
    ])
