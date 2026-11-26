import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import TimerAction


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    serial_port = LaunchConfiguration("serial_port")
    lower_mcu_serial_port = LaunchConfiguration("lower_mcu_serial_port")
    map_yaml_path = LaunchConfiguration(
        "map",
        default=PathJoinSubstitution([FindPackageShare("dianshu_nav"), "maps", "default_map.yaml"]),
    )
    nav2_param_file = LaunchConfiguration(
        "params_file",
        default=PathJoinSubstitution([FindPackageShare("dianshu_nav"), "param", "nav2.yaml"]),
    )
    slam_params_file = LaunchConfiguration(
        "slam_params_file",
        default=PathJoinSubstitution([FindPackageShare("dianshu_nav"), "param", "slam_online_async.yaml"]),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_708a192cb573ef1189f6c88c8fcc3fa0-if00-port0"),
            DeclareLaunchArgument("lower_mcu_serial_port", default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "map", default_value=map_yaml_path, description="Full path to map file to load"
            ),
            DeclareLaunchArgument(
                "params_file", default_value=nav2_param_file, description="Full path to param file to load"
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=slam_params_file,
                description="Full path to slamtoolbox async param file to load",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("dianshu_base"), "/launch", "/robot_bringup_launch.py"]
                ),
                launch_arguments={
                    "serial_port": serial_port,
                    "lower_mcu_serial_port": lower_mcu_serial_port,
                }.items(),
            ),
            TimerAction(
                period=3.0,  # 5秒延时
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [FindPackageShare("nav2_bringup"), "/launch", "/navigation_launch.py"]
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "params_file": nav2_param_file,
                        }.items(),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [FindPackageShare("slam_toolbox"), "/launch", "/online_async_launch.py"]
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "slam_params_file": slam_params_file,
                        }.items(),
                    ),
                ],
            ),
        ]
    )
