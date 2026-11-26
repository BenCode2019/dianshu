from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    launch_arguments = [
        # DeclareLaunchArgument("lower_mcu_serial_port", default_value="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"),
        DeclareLaunchArgument("lower_mcu_serial_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument(
            "imu_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("dianshu_base"), "config", "imu.yaml"]
            ),
            description="IMU sensitivity, bias and covariance parameters",
        ),
        DeclareLaunchArgument(
            "drive_params_file",
            default_value=PathJoinSubstitution([FindPackageShare("dianshu_base"), "config", "drive.yaml"]),
            description="Angular command and wheel odometry calibration",
        ),
        DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
        DeclareLaunchArgument(
            "publish_odom_tf", default_value="true",
            description="Disable when the EKF owns odom -> base_link",
        ),
        DeclareLaunchArgument(
            "publish_imu_tf", default_value="true",
            description="Enable after setting the measured base_link -> IMU mounting transform",
        ),
    ]
    # Translation is in metres; mounting roll/pitch/yaw are in radians.
    for name in ("imu_x", "imu_y", "imu_z", "imu_roll", "imu_pitch", "imu_yaw"):
        launch_arguments.append(DeclareLaunchArgument(name, default_value="0.0"))

    dianshu_base = Node(
        package="dianshu_base",
        executable="dianshu_base",
        name="dianshu_base_node",
        parameters=[
            LaunchConfiguration("imu_params_file"),
            LaunchConfiguration("drive_params_file"),
            {
                "lower_mcu_serial_port": LaunchConfiguration("lower_mcu_serial_port"),
                "imu_frame_id": LaunchConfiguration("imu_frame_id"),
                "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
            }
        ],
        # arguments="--ros-args --log-level debug".split(" "),
    )
    tf_baselink_imu_static_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_imu",
        condition=IfCondition(LaunchConfiguration("publish_imu_tf")),
        arguments=[
            "--x", LaunchConfiguration("imu_x"),
            "--y", LaunchConfiguration("imu_y"),
            "--z", LaunchConfiguration("imu_z"),
            "--roll", LaunchConfiguration("imu_roll"),
            "--pitch", LaunchConfiguration("imu_pitch"),
            "--yaw", LaunchConfiguration("imu_yaw"),
            "--frame-id", "base_link",
            "--child-frame-id", LaunchConfiguration("imu_frame_id"),
        ],
    )
    # base_footprint->base_link
    tf_basefootprint_baselink_static_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_base_footprint",
        arguments=[
            "--x", "-0.025", "--y", "0.0", "--z", "-0.025",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
            "--frame-id", "/base_link", "--child-frame-id", "/base_footprint",
        ],
    )

    return LaunchDescription(
        launch_arguments + [
            dianshu_base, tf_basefootprint_baselink_static_node,
            tf_baselink_imu_static_node,
        ]
    )
