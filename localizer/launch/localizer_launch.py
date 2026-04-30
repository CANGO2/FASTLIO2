import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("localizer"), "rviz", "localizer.rviz"]
    )
    localizer_config_path = PathJoinSubstitution(
        [FindPackageShare("localizer"), "config", "localizer.yaml"]
    )
    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "lio.yaml"]
    )

    # 3초 후 map 자동 로드
    load_map_cmd = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/localizer/relocalize',
                    'interface/srv/Relocalize',
                    "{pcd_path: '/home/nuc/GlobalMap2.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
                ],
                output='screen'
            )
        ]
    )

    # 4초 후 initialpose 브릿지 실행 (map 로드 후)
    bridge_cmd = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', '/home/nuc/initialpose_bridge.py'],
                output='screen'
            )
        ]
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="lio_node",
                name="lio_node",
                output="screen",
                parameters=[
                    {"config_path": lio_config_path.perform(launch.LaunchContext())}
                ],
            ),
            launch_ros.actions.Node(
                package="localizer",
                namespace="localizer",
                executable="localizer_node",
                name="localizer_node",
                output="screen",
                parameters=[
                    {
                        "config_path": localizer_config_path.perform(
                            launch.LaunchContext()
                        )
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="localizer",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            ),
            load_map_cmd,
            bridge_cmd,
        ]
    )
