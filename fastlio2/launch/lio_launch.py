import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"]
    )

    config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )


    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="fastlio2",
                executable="dynamic_object_filter_node",
                name="dynamic_object_filter_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/points_raw",
                        "output_topic": "/points_raw_static",
                        "enabled": True,
                        "remove_min_range": 0.2,
                        "remove_max_range": 1.5,
                        "remove_min_z": 0.1,
                        "remove_max_z": 1.8,
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[{"config_path": config_path.perform(launch.LaunchContext())}]
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="fastlio2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            ),
        ]
    )
