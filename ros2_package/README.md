## 项目概述

本项目是基于 ROS2 的**六自由度机械臂控制系统**，通过 FDCAN 总线驱动达秒（Damiao）系列电机，采用 MIT 阻抗控制模式，支持真实硬件控制、Mujoco 仿真，控制方面可以采用手柄控制末端关节，拥有重力补偿功能。


## 环境配置

运行在 Ubuntu 24.04 LTS 系统上，ROS2 Jazzy 版本。

### 依赖安装

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-joy
sudo apt install ros-jazzy-pinocchio
sudo apt install libusb-1.0-0-dev
```

### 可视化工具（可选）

```bash
sudo apt install ros-jazzy-plotjuggler
```

## 编译与运行

```bash
# 编译所有包（在工作空间根目录 ~/Project/ros2_Ragtime 下执行）
cd ~/Project/ros2_Ragtime
colcon build --symlink-install
source install/setup.bash
```

### 控制方案切换

配置文件：`robot_description/urdf/robot_1.ros2_control.xacro` 的 `<plugin>` 部分：

| 插件 | 用途 |
|------|------|
| `mock_components/GenericSystem` | 模拟组件，用于代码调试（当前默认） |
| `mobile_base_hardware/MobileBaseHardwareInterface` | 真实硬件，需接 USB2CAN 连接电机 |
| `mujoco_ros2_control/MujocoSystemInterface` | Mujoco 仿真 |

### 启动命令

```bash
# 运行控制节点（真实/模拟硬件）
ros2 launch robot_bringup robot_controllers.launch.py

# 运行 Mujoco 仿真节点
ros2 launch robot_bringup robot_mujoco.launch.py

# 运行手柄 + Pinocchio 逆运动学控制
ros2 launch robot_bringup my_robot_pin.launch.py
```

## 软件包架构

```
手柄输入 → PinocchioInverseNode → arm_MIT_command_controller/commands
                                              ↓
                               MotorController（position_ff / gravity_comp）
                                              ↓
                               MobileBaseHardwareInterface（250 Hz 循环）
                                              ↓  
                               robot_dm_driver（CAN 协议，CANFD）
                                              ↓
                               达秒电机（MIT 阻抗：kp·Δq + kd·Δdq + τ_ff）
```

### 各软件包简要说明（多数软件包内部都写了README文档）

| 软件包 | 功能 |
|--------|------|
| `robot_description` | 包含 URDF、xacro、xml机械臂描述文件，STL网格模型，主要需要修改的是robot_1.ros2_control.xacro，可以修改MIT增益参数、控制模式|
| `robot_dm_driver` | 包含[达秒官方的封装闭源库](https://gitee.com/kit-miao/dm-tools/tree/master/DM_DeviceSDK/C&C++) `libdm_device.so`，pub_user.h API 接口, src里面写的是对API的实现包含MIT控制模式的实现，接收电机反馈数据。|
| `my_robot_hardware` | 介于控制器和底层API之间，实现了 ROS2 硬件接口，起到的主要作用是设置电机ID、发送控制器的指令、接收电机反馈数据传送给控制器(如：关节广播控制器) |
| `robot_controllers` | 含有两种控制模式，一个是手柄控制，接收上层节点的关节速度指令，积分为位置，另一种是接收位置指令，通过pinocchio的函数实现重力补偿，此外控制器还包含了低通滤波。控制器控制模式可以通过配置文件修改：`robot_bringup/config/ros2_controllers.yaml` |
| `robot_pinocchio` | DLS 逆运动学节点，手柄轴输入 → 关节速度指令，250 Hz 定时发布 |
| `robot_bringup` | Launch 启动文件及控制器 YAML 配置文件，和一个rviz保存的配置文件 |


### 电机驱动（`robot_dm_driver`）
- 只要是达秒拥有MIT控制的电机应该是都可以用的，本项目仅使用了 DM_4340 和 DM_4310 系列
- 关于MIT控制详细内容请参考[达秒官方文档](https://gl1po2nscb.feishu.cn/wiki/Y3OEwMr4GivZU9kZqkjctmGinye)
- CANFD 双波特率：标称 1 Mbps，数据段 5 Mbps
- `libdm_device.so` 为[达秒官方的闭源二进制库](https://gitee.com/kit-miao/dm-tools/tree/master/DM_DeviceSDK/C&C++)，位于 `robot_dm_driver/lib/`，运行时必须存在
- 电机状态由互斥锁保护，读取时使用 `dm_motor_snapshot(idx, &q, &dq, &tau)` 保证线程安全


### 硬件接口（`my_robot_hardware`）
- 类名：`MobileBaseHardwareInterface`（命名有误导性，实为机械臂接口）
- 插件描述文件：`my_robot_hardware_interface.xml`
- **`on_init()`**：从 URDF `<param>` 读取每关节 kp/kd，创建 damiao handle，查找 USB2CAN 设备
- **`on_configure()`**：打开设备，打印固件版本和序列号
- **`on_activate()`**：初始化 CANFD 通道，注册 6 个电机（CAN ID 0x01–0x06），开启通道，使能控制，等待最多 500 ms 接收首次反馈
- **`read()`**：突变滤波（偏差 > 0.2 rad 则保持上帧值）+ 低通滤波（alpha=0.4）
- **`write()`**：调用 `dm_control_mit()` 透传 pos/vel/eff，含 NaN 保护（跳过异常帧）

### 电机控制器（`robot_controllers`）
- 命名空间：`robot_controller::MotorController`（注意：非 `robot_controllers`）
- 插件：`robot_controller::MotorController`
- **`position_ff` 模式**（默认）：读取速度指令（`data[i*3+1]`），低通滤波（alpha=0.4），积分出位置目标，dt 限幅 [0.5 ms, 10 ms]
- **`gravity_comp` 模式**：使用 Pinocchio `computeGeneralizedGravity()` 计算重力力矩，pos=当前位置，vel=0，eff=τ_gravity
- 订阅话题：`arm_MIT_command_controller/commands`（`Float64MultiArray`，18 元素，每关节 `[pos, vel, eff]`）

### Pinocchio 逆运动学（`robot_pinocchio`）
- 节点名：`pinocchio_inverse_node`
- 使用阻尼最小二乘法（DLS）雅可比伪逆，`IK_DT=0.04`，奇异点阈值 `IK_EPS=0.01`
- 定时器周期 4 ms（约 250 Hz）发布关节速度指令
- 手柄左摇杆（LEFTX/LEFTY）→ 末端笛卡尔速度 → DLS 映射为 6 个关节速度,本项目没有添加末端姿态控制，如果需要你可以尝试添加；
- 速度缩放因子 `factor=0.03`，奇异度指标 `ratio`（< 1 表示接近奇异）
- 订阅 `joint_states`关节状态，发布到 `arm_MIT_command_controller/commands`控制器
- URDF 路径通过 `ament_index_cpp::get_package_share_directory("robot_description")` 动态获取


### 控制器配置
- 更新频率：250 Hz（在 `robot_bringup/config/ros2_controllers.yaml` 中设置）
- 各关节 kp/kd 增益定义在 `robot_description/urdf/robot_1.ros2_control.xacro`
- **注意**：`ros2_controllers.yaml` 中 URDF 路径为绝对路径，工作空间移动后需同步修改

## 致谢
