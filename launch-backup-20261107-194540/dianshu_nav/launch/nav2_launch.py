from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_fake_base = LaunchConfiguration("use_fake_base", default="false")
    use_lidar = LaunchConfiguration("use_lidar", default="true")
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB1")
    lower_mcu_serial_port = LaunchConfiguration("lower_mcu_serial_port", default="/dev/ttyUSB0")
    map_yaml_path = LaunchConfiguration(
        "maps",
        default=PathJoinSubstitution([FindPackageShare("dianshu_nav"), "maps", "map.yaml"]),
    )
    nav2_param_path = LaunchConfiguration(
        "params_file",
        default=PathJoinSubstitution([FindPackageShare("dianshu_nav"), "param", "nav2.yaml"]),
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("dianshu_base"), "/launch", "/robot_bringup_launch.py"]
                ),
                launch_arguments={
                    "serial_port": serial_port,
                    "lower_mcu_serial_port": lower_mcu_serial_port,
                    "use_fake_base": use_fake_base,
                    "use_lidar": use_lidar,
                }.items(),
            ),
            DeclareLaunchArgument("use_fake_base", default_value=use_fake_base),
            DeclareLaunchArgument("use_lidar", default_value=use_lidar),
            DeclareLaunchArgument("serial_port", default_value=serial_port),
            DeclareLaunchArgument(
                "lower_mcu_serial_port", default_value=lower_mcu_serial_port
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "map", default_value=map_yaml_path, description="Full path to map file to load"
            ),
            DeclareLaunchArgument(
                "params_file", default_value=nav2_param_path, description="Full path to param file to load"
            ),
            TimerAction(
                period=3.0,  # 5秒延时
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [FindPackageShare("nav2_bringup"), "/launch", "/bringup_launch.py"]
                        ),
                        launch_arguments={
                            "map": map_yaml_path,
                            "use_sim_time": use_sim_time,
                            "params_file": nav2_param_path,
                            "autostart": "true",
                            # Keep the original composed Nav2 startup.  The
                            # cmd_vel relay below connects its raw controller
                            # output to the velocity smoother input.
                            # navigation_launch.py evaluates this with Python
                            # expression syntax, which requires capitalized
                            # boolean literals.
                            "use_composition": "True",
                        }.items(),
                    ),
                ],
            ),
            Node(
                package="dianshu_base",
                executable="cmd_vel_relay",
                name="cmd_vel_relay",
                output="screen",
            ),
        ]
    )
