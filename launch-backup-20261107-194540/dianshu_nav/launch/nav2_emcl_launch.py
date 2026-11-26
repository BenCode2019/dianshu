"""Start the robot, EMCL2 localization, and the Nav2 navigation stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map1")
    params_file = LaunchConfiguration("params_file")
    use_fake_base = LaunchConfiguration("use_fake_base")
    use_lidar = LaunchConfiguration("use_lidar")
    serial_port = LaunchConfiguration("serial_port")
    lower_mcu_serial_port = LaunchConfiguration("lower_mcu_serial_port")
    emcl_params = LaunchConfiguration("emcl_params_file")

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("dianshu_base"), "launch", "robot_bringup_launch.py"])
        ),
        launch_arguments={
            "serial_port": serial_port,
            "lower_mcu_serial_port": lower_mcu_serial_port,
            "use_fake_base": use_fake_base,
            "use_lidar": use_lidar,
        }.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("emcl2"), "launch", "emcl_localization.launch.py"])
        ),
        launch_arguments={
            "map": map_file,
            "use_sim_time": use_sim_time,
            "emcl_yaml_file": emcl_params,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "true",
            "use_composition": "True",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_fake_base", default_value="false"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument("lower_mcu_serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution([FindPackageShare("dianshu_nav"), "maps", "map.yaml"]),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([FindPackageShare("dianshu_nav"), "param", "nav2.yaml"]),
        ),
        DeclareLaunchArgument(
            "emcl_params_file",
            default_value=PathJoinSubstitution([FindPackageShare("emcl2"), "config", "emcl2.param.yaml"]),
        ),
        base,
        localization,
        TimerAction(period=3.0, actions=[navigation]),
        Node(package="dianshu_base", executable="cmd_vel_relay", name="cmd_vel_relay", output="screen"),
    ])
