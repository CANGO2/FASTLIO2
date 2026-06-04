from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    hesai_share = get_package_share_directory("hesai_lidar")
    fastlio_share = get_package_share_directory("fastlio2")
    localizer_share = get_package_share_directory("localizer")
    safety_share = get_package_share_directory("safety_checker")

    lidar_correction_file = os.path.join(hesai_share, "config", "PandarXT-16.csv")
    lio_config_path = os.path.join(fastlio_share, "config", "lio.yaml")
    localizer_config_path = os.path.join(localizer_share, "config", "localizer.yaml")
    safety_config_path = os.path.join(safety_share, "config", "params.yml")
    rviz_cfg = os.path.join(localizer_share, "rviz", "localizer.rviz")

    perception_container = ComposableNodeContainer(
        name="perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="hesai_lidar",
                plugin="hesai_lidar::HesaiLidarClient",
                name="hesai_node",
                namespace="",
                parameters=[
                    {"pcap_file": ""},
                    {"server_ip": "192.168.123.20"},
                    {"lidar_recv_port": 2368},
                    {"gps_port": 10110},
                    {"start_angle": 0.0},
                    {"lidar_type": "PandarXT-16"},
                    {"frame_id": "hesai_lidar"},
                    {"pcldata_type": 0},
                    {"publish_type": "points"},
                    {"timestamp_type": "realtime"},
                    {"data_type": ""},
                    {"lidar_correction_file": lidar_correction_file},
                    {"multicast_ip": ""},
                    {"coordinate_correction_flag": False},
                    {"fixed_frame": ""},
                    {"target_frame": ""},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="fastlio2",
                plugin="LIONode",
                name="lio_node",
                namespace="fastlio2",
                parameters=[{"config_path": lio_config_path}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="safety_checker",
                plugin="ObstacleSafetyNode",
                name="safety_checker",
                namespace="",
                parameters=[safety_config_path],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    imu_publisher_node = Node(
        package="go2_demo",
        executable="imu_publisher",
        name="imu_publisher",
        output="screen",
    )

    localizer_node = Node(
        package="localizer",
        namespace="localizer",
        executable="localizer_node",
        name="localizer_node",
        output="screen",
        parameters=[{"config_path": localizer_config_path}],
    )

    pose_to_navigation_node = Node(
        package="pose_to_navigation",
        executable="pose_to_navigation",
        name="pose_to_navigation",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        namespace="localizer",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
    )

    load_map_cmd = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/localizer/relocalize",
                    "interface/srv/Relocalize",
                    "{pcd_path: '/home/nuc/GlobalMap2.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}",
                ],
                output="screen",
            )
        ],
    )

    bridge_cmd = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    "/home/nuc/ros2_ws/src/FASTLIO2/localizer/src/initialpose_bridge.py",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            perception_container,
            imu_publisher_node,
            localizer_node,
            pose_to_navigation_node,
            rviz_node,
            load_map_cmd,
            bridge_cmd,
        ]
    )
