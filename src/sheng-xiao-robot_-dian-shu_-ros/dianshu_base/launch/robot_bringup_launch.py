from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

# 指定的雷达串口号


def generate_launch_description():
    # 雷达启动部分
    sl_lidar_serial_port_launch_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_708a192cb573ef1189f6c88c8fcc3fa0-if00-port0",  # rasp uart0
        description="Specifying usb port to connected lidar",
    )
    use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value="true",
        description="Start the SLLidar driver and base_link to laser TF",
    )
    
    # 雷达launch文件,使用serial_port参数
    sl_lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution([FindPackageShare("sllidar_ros2"), "launch", "sllidar_c1_launch.py"]),
            ]
        ),
        launch_arguments={
            "serial_port": LaunchConfiguration("serial_port"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_lidar")),
    )
    # 此处静态tf的姿态表达为四元数[x y z w],0 0 0 1表示0 0 0;0 0 1 0 表示0 0 -pi
    # base_link->laser
    tf_baselink_laser_static_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_laser",
        condition=IfCondition(LaunchConfiguration("use_lidar")),
        arguments="--x -0.035 --y 0.0 --z 0.13 --qx 0 --qy 0 --qz 1 --qw 0 --frame-id /base_link --child-frame-id /laser".split(
            " "
        ),
    )

    # 底盘驱动启动部分
    lower_mcu_serial_port_arg = DeclareLaunchArgument(
        "lower_mcu_serial_port",
        default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",  # rasp uart2
        description="Specifying usb port to connected servo",
    )
    use_fake_base_arg = DeclareLaunchArgument(
        "use_fake_base",
        default_value="false",
        description="Use a local odometry simulator when the B20 lower controller is absent",
    )
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution([FindPackageShare("dianshu_base"), "launch", "base_launch.py"]),
            ]
        ),
        launch_arguments={
            "lower_mcu_serial_port": LaunchConfiguration("lower_mcu_serial_port"),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_fake_base")),
    )
    fake_base = Node(
        package="dianshu_base",
        executable="fake_base",
        name="fake_base",
        condition=IfCondition(LaunchConfiguration("use_fake_base")),
    )
    return LaunchDescription(
        [
            sl_lidar_serial_port_launch_arg,
            use_lidar_arg,
            sl_lidar_driver,
            tf_baselink_laser_static_node,
            lower_mcu_serial_port_arg,
            use_fake_base_arg,
            base_launch,
            fake_base,
        ]
    )
