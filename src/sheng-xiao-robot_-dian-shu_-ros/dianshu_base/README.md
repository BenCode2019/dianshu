# 底盘与 IMU 数据上报

底盘节点共用一个 B20 串口接收轮速和板载 MPU6050 数据，发布 `/odom` 与
`/imu/data_raw`，继续接受 `/cmd_vel` 和 `/camera_pos`。

## 上报协议

串口为 115200 baud，接收的是二进制字节。已通过 `/dev/ttyUSB0` 实测两种帧交替上报，
各约 10 Hz；帧长与公开下位机固件中的 `upload_imu_rawdata()` 一致。

| 类型 | 帧格式 | 数值编码 |
| --- | --- | --- |
| 轮速，7 字节 | `FF 01 LEFT_H LEFT_L RIGHT_H RIGHT_L FE` | 大端，最高位为符号，其余 15 位为绝对值，单位 pps |
| IMU，15 字节 | `FF 04 GX_H GX_L GY_H GY_L GZ_H GZ_L AX_H AX_L AY_H AY_L AZ_H AZ_L FE` | 大端，6 个二进制补码 `int16`，原始传感器计数 |

注意 IMU 前三项是陀螺仪 XYZ 原始计数，不是欧拉角。载荷中的 `FF`、`FE` 是合法数值，
接收器按照类型对应的帧长判断帧尾，支持拆包、粘包和坏帧后的重新对齐。
协议没有校验和，无法检测仍具有合法帧头和帧尾的载荷损坏。

## ROS 输出和标定参数

`/imu/data_raw` 类型为 `sensor_msgs/msg/Imu`，默认 `frame_id=imu_link`。
角速度为 rad/s，加速度为 m/s²，坐标轴保持 MPU6050 原始右手传感器坐标系。
加速度保留重力对应的比力；传感器 Z 轴朝上、静止时 Z 应接近 +9.80665 m/s²。
固件没有上报姿态，`orientation_covariance[0]` 设置为 `-1`。
时间戳为完整帧在 ROS 端处理时的 ROS 时间，协议不含 MCU 采样时间戳。

配置文件为 `config/imu.yaml`，通过 `imu_params_file` 启动参数指定。
参数在节点启动时读取，修改配置后需要重启底盘节点。
默认灵敏度来自公开固件 `mpu6050_init()` 的 ±2000°/s 和 ±2g 设置：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `imu_gyro_lsb_per_dps` | `16.4` | 陀螺仪每 °/s 对应计数 |
| `imu_accel_lsb_per_g` | `16384.0` | 每 g 对应加速度计数 |
| `imu_angular_velocity_bias` | `[0.0, 0.0, 0.0]` | 传感器 XYZ 零偏，rad/s，换算后减去 |
| `imu_linear_acceleration_bias` | `[0.0, 0.0, 0.0]` | 传感器 XYZ 零偏，m/s²，换算后减去 |
| `imu_angular_velocity_covariance_diagonal` | `[0.0, 0.0, 0.0]` | XYZ 方差，(rad/s)² |
| `imu_linear_acceleration_covariance_diagonal` | `[0.0, 0.0, 0.0]` | XYZ 方差，(m/s²)² |

协方差全零表示未知，需要在 EKF 使用前测量和配置，不代表没有噪声。
静止采样可用于估计陀螺仪零偏和噪声；加速度零偏需结合已知方向的重力标定，
不要把静止时完整的 Z 轴重力值当作零偏扣除。若你修改过固件量程，请同步修改灵敏度。

## 启动与检查

在工作空间中执行：

```bash
colcon build --packages-select dianshu_base
source install/setup.bash
ros2 launch dianshu_base base_launch.py lower_mcu_serial_port:=/dev/ttyUSB0
```

另一个终端检查：

```bash
source install/setup.bash
ros2 topic echo /imu/data_raw --once
ros2 topic hz /imu/data_raw
```

同一串口应只有一个底盘驱动读数据。

没有 B20 下位机时，可用 `use_fake_base:=true` 启动一个仅用于 Nav2 软件测试的
差速底盘模型。它根据 `/cmd_vel` 积分生成 `/odom` 和 `odom -> base_link`，不代表真实
电机或编码器反馈：

```bash
ros2 launch dianshu_nav nav2_launch.py use_fake_base:=true
```

没有雷达时可用 `use_lidar:=false`，这样不会打开雷达串口或发布 `base_link -> laser`。
此时 AMCL 没有激光观测，不能完成真实地图定位；该选项只用于检查节点启动和控制器
话题链路，不能据此判断实际避障导航能力。

## EKF 接入前的配置

安装外参无法由静止串口数据确定，所以 `publish_imu_tf` 默认关闭。
如果已有 URDF 发布 `base_link -> imu_link`，继续使用该变换。
否则根据实际安装测量 `imu_x`、`imu_y`、`imu_z`（米）和
`imu_roll`、`imu_pitch`、`imu_yaw`（弧度），通过启动参数设置并启用
`publish_imu_tf:=true`。这些量描述 IMU 传感器坐标系相对于 `base_link` 的安装姿态。
`imu_frame_id` 同时用于消息和该静态 TF 的子坐标系，默认 `imu_link`。

平面车可先使用 IMU 的 Z 轴角速度与轮速融合，姿态字段不可选作融合输入。
在考虑融合加速度前，需要处理零偏、重力及安装姿态；此原始消息不含姿态估计，
不能直接依赖其 orientation 进行重力补偿。
EKF 发布 `odom -> base_link` 时，用 `publish_odom_tf:=false` 关闭底盘的同名 TF；
底盘仍会发布 `/odom` 供融合使用。

参考：[下位机固件](https://gitee.com/shengxiaorobot/sheng-xiao-robot_-dian-shu_-controller)、
[MPU6050 数据手册](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf)、
[ROS IMU 消息定义](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg)、
[REP-145](https://github.com/ros-infrastructure/rep/blob/master/rep-0145.rst)。


## 2026-11-07 旋转标定

建图和导航均通过 `base_launch.py` 自动加载 `config/drive.yaml`。
`command_angular_scale_ccw/cw` 只修正角速度下发；
`odom_angular_scale_ccw/cw` 只修正编码器的角速度及积分转角。
两者不能根据一次整圈计时一起修改。正方向为 ROS 逆时针。

实测低速轮速存在约 50 PPS 阶梯：185/195 PPS 下发反馈相近，200 PPS 才进入下一档。
`wheel_speed_quantum_pps: 50.0` 以固定 20 Hz 在相邻档位分配时间，
保持平均目标轮速；单个时刻仍存在舵机速度分辨率，不能保证瞬时角速度完全一致。
设为 0.0 可以关闭分时补偿（新固件修复分辨率后应重新标定）。

里程计按单调时钟实测间隔积分，同一批轮速帧平分接收间隔。
首次接收或超过 0.5 秒的反馈间断不外推位姿；协议没有硬件采样时间戳，
长时间串口丢帧无法精确恢复轨迹。控制指令超过 `cmd_vel_timeout` 秒未更新即停车，
因此持续运动须连续发布 `/cmd_vel`（建议 10 Hz 以上），不能只发布一条非零指令。

标定记录、雷达扫描、IMU、指令及分析脚本保存在工作空间 `calibration-work/`。
这些系数仅验证原地低速旋转；轮径、直线距离和负载/地面变化需要另行验证。


最终运行方案保留编码器里程计，IMU 仅用于标定交叉核对，不作为航向积分源。
旋转校准不能替代直线距离标定，分时补偿后的角速度在短时间窗口内仍会有波动。
软件验证请使用独立 DDS 域，避免伪串口测试数据混入真实 `/odom`：

```bash
ROS_DOMAIN_ID=90 python3 -m pytest -q src/sheng-xiao-robot_-dian-shu_-ros/dianshu_base/test/test_protocol.py src/sheng-xiao-robot_-dian-shu_-ros/dianshu_base/test/test_imu_driver.py
```

最终整圈复测（约 0.185302 rad/s，33.91 秒，雷达扫描匹配参考）：

| 方向 | 雷达实际转角 | 里程计累计转角 | 差异 |
|---|---:|---:|---:|
| 逆时针 | 358.42° | 359.99° | 1.57° |
| 顺时针 | 362.02° | 359.68° | 2.34° |
