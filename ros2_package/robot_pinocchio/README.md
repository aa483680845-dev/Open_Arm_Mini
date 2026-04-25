# robot_pinocchio

基于 Pinocchio 的六自由度机械臂逆运动学节点，将手柄摇杆输入转换为关节速度指令，以 250 Hz 频率发布给控制器。

## 功能概述

- 从 URDF 加载机器人模型，使用 Pinocchio 计算运动学
- 读取手柄摇杆输入，映射为末端执行器的笛卡尔速度
- 采用阻尼最小二乘法（DLS）求解雅可比伪逆，将笛卡尔速度转换为关节速度
- 通过方向键（D-pad）实时调节速度缩放因子
- 在收到第一帧关节状态前不发布任何指令，避免零位跳变

## 文件结构

```
robot_pinocchio/
├── include/robot_pinocchio/
│   └── pinocchio_inverse.hpp     # IK 求解器类声明，手柄/关节数据结构
├── src/
│   ├── pinocchio_inverse.cpp     # DLS 逆运动学实现
│   └── pinocchio_main.cpp        # ROS2 节点：订阅、发布、定时器
├── CMakeLists.txt
└── package.xml
```

## 话题

| 方向 | 话题名 | 消息类型 | 说明 |
|------|--------|----------|------|
| 订阅 | `joint_states` | `sensor_msgs/JointState` | 读取当前各关节位置 |
| 订阅 | `joy` | `sensor_msgs/Joy` | 读取手柄摇杆和按键输入 |
| 发布 | `arm_MIT_command_controller/commands` | `std_msgs/Float64MultiArray` | 发布 18 元素关节指令（每关节 `[pos, vel, eff]`） |

发布的指令中：`pos=0`，`vel=关节速度`，`eff=0`，由下游 `MotorController` 在 `position_ff` 模式下积分位置目标。

## 手柄映射

| 手柄输入 | 轴索引 | 作用 |
|----------|--------|------|
| 左摇杆 Y 轴（前后） | LEFTY (1) | 末端 X 方向平移速度 |
| 左摇杆 X 轴（左右） | LEFTX (0) | 末端 Y 方向平移速度 |
| 右摇杆 Y 轴（前后） | RIGHTY (4) | 末端 Z 方向平移速度 |
| D-pad 上 | DPAD_Y > 0.5 | 速度缩放因子 +0.01 |
| D-pad 下 | DPAD_Y < -0.5 | 速度缩放因子 -0.01 |

> 当前版本仅控制末端平移，姿态速度分量固定为零。如需添加姿态控制，可将右摇杆 X 轴或扳机等映射到 `v_world(3~5)`。

## 关键参数

| 参数 | 位置 | 默认值 | 说明 |
|------|------|--------|------|
| `IK_DT` | `pinocchio_inverse.hpp` | `0.04` | IK 积分步长（s） |
| `IK_EPS` | `pinocchio_inverse.hpp` | `0.01` | 奇异点判断阈值 |
| `factor` | `pinocchio_inverse.hpp` | `0.03` | 速度缩放因子，范围 [0.01, 0.05] |
| `damp` | `pinocchio_inverse.cpp` | `1e-4` | DLS 阻尼系数，防止奇异点下速度爆炸 |
| 定时器周期 | `pinocchio_main.cpp` | `4 ms` | 控制频率约 250 Hz |

## 算法说明

### DLS 逆运动学

核心思路：给定末端期望笛卡尔速度 $\mathbf{v}$，求满足 $J\dot{q} = \mathbf{v}$ 的关节速度 $\dot{q}$。

直接求逆在奇异点附近数值不稳定，因此采用**阻尼最小二乘法**（Damped Least Squares）：

$$\dot{q} = J^T (J J^T + \lambda^2 I)^{-1} \mathbf{v}$$

其中 $\lambda^2 = 10^{-4}$ 为阻尼系数。

实现步骤（`ik_solve`）：

1. 以当前关节角 `q` 做正运动学，更新末端位姿
2. 将世界坐标系下的笛卡尔速度转换到末端局部坐标系（`oMi.actInv`）
3. 用 `pinocchio::computeJointJacobian` 计算关节 6 的几何雅可比 $J \in \mathbb{R}^{6\times6}$
4. 组装 $JJ^T + \lambda I$ 并用 LDLT 分解求解线性方程组
5. 输出关节速度 $\dot{q}$，存入 `v_`

### 多线程设计

节点使用 `MultiThreadedExecutor`，三个回调各属独立的 `MutuallyExclusive` 回调组：

- **`joint_state` 回调**：更新关节角缓冲，设置 `joint_state_received_` 标志
- **`joy` 回调**：更新手柄数据缓冲
- **定时器回调**：无锁快照两个缓冲 → 调用 IK 求解 → 发布指令

两个缓冲各有独立互斥锁，定时器取快照时加锁时间极短，避免长时间阻塞。

## 与其他节点的数据流

```
sensor_msgs/Joy (手柄驱动)
        │  /joy
        ▼
PinocchioInverseNode
        │  订阅 joint_states（关节反馈）
        │  DLS IK 求解（250 Hz）
        │  /arm_MIT_command_controller/commands
        ▼
MotorController（position_ff 模式）
        │  积分位置目标 + 低通滤波
        ▼
MobileBaseHardwareInterface → 达秒电机（MIT 阻抗控制）
```

## 启动

```bash
# 同时启动控制器和 Pinocchio IK 节点
ros2 launch robot_bringup my_robot_pin.launch.py
```

需要确保：
- 手柄已连接并由 `ros-jazzy-joy` 的 `joy_node` 发布 `/joy` 话题
- `robot_description` 包下存在编译好的 `robot_1.urdf`（由 xacro 生成）
- 控制器已加载并处于 `position_ff` 模式（见 `robot_bringup/config/ros2_controllers.yaml`）

## 依赖

```bash
sudo apt install ros-jazzy-pinocchio
sudo apt install ros-jazzy-joy
```
