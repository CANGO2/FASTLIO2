from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # 1. Hesai LiDAR 런치
    hesai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hesai_lidar'),
                'launch',
                'hesai_lidar_launch.py'
            )
        )
    )

    # 2. IMU Publisher 노드
    imu_publisher_node = Node(
        package='go2_demo',
        executable='imu_publisher',
        name='imu_publisher',
        output='screen'
    )

    # 3. Localizer 런치
    localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('localizer'),
                'launch',
                'localizer_launch.py'
            )
        )
    )

    # 4. Pose to Navigation 노드
    pose_to_navigation_node = Node(
        package='pose_to_navigation',
        executable='pose_to_navigation',
        name='pose_to_navigation',
        output='screen'
    )

    return LaunchDescription([
        hesai_launch,
        imu_publisher_node,
        localizer_launch,
        pose_to_navigation_node,
    ])
