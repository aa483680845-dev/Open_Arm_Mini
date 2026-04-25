# robot_controllers

## 概述

本包实现了一个自定义 ROS2 控制器 `MotorController`，用于驱动六自由度机械臂的达秒（Damiao）系列电机，支持**位置前馈（position_ff）**和**重力补偿（gravity_comp）**两种控制模式。

### 为什么不用官方控制器？

ROS2 [官方控制器库](https://github.com/ros-controls/ros2_controllers)提供的控制器通常是单输入单输出，或多输入单输出的形式。本项目需要**同时输出位置、速度、力矩**三路指令（对应达秒 MIT 阻抗控制模式的三个参数），官方库无法直接满足这一需求。

解决思路有两种：

| 方案 | 描述 | 优缺点 |
|------|------|--------|
| **方案一（本包采用）** | 自定义控制器，算法全部封装在控制器内，硬件层仅负责数据透传 | 算法集中，复用性好，结构清晰 |
| 方案二 | 使用官方位置控制器，在 `hardware_interface` 内部叠加重力补偿 | 实现简单，但算法分散在硬件层，不利于复用 |

---

## 控制模式

通过配置文件中的 `control_mode` 参数切换，配置文件路径：`robot_bringup/config/ros2_controllers.yaml`。

### `position_ff`（默认）

接收上游发布的关节**速度指令**，经低通滤波后积分为位置目标，同时将滤波后的速度作为前馈量发送给硬件。

```
速度指令 (vel_cmd)
    → 低通滤波（alpha = 0.4）
    → 积分（dt 限幅 [0.5ms, 10ms]）
    → 位置目标 (pos_cmd)
    → 硬件接口 [pos_cmd, vel_filtered, 0.0]
```

### `gravity_comp`

读取当前关节角，利用 Pinocchio `computeGeneralizedGravity()` 实时计算各关节重力补偿力矩，并将当前位置作为目标位置发送，使机械臂保持当前姿态的同时补偿重力。

```
当前关节角 q (来自 state_interfaces)
    → Pinocchio RNEA → τ_gravity = g(q)
    → 硬件接口 [q_now, 0.0, τ_gravity]
```

> 发送 `pos = q_now`，使 MIT 控制器中的 `kp*(q_now - q_actual) ≈ 0`，位置增益几乎不出力；
> 发送 `vel = 0`，MIT 控制器中的 `-kd * dq_actual` 提供速度阻尼。

---

## 接口说明

### 订阅话题

| 话题 | 消息类型 | 说明 |
|------|----------|------|
| `arm_MIT_command_controller/commands` | `std_msgs/Float64MultiArray` | 上游指令，18 元素，每关节 `[pos, vel, eff]` 排列（`position_ff` 模式仅使用 `vel`，即 `data[i*3+1]`） |

### 命令接口（写入硬件）

每个关节输出三路命令接口：`position`、`velocity`、`effort`。

### 状态接口（读取硬件）

每个关节读取三路状态接口：`position`、`velocity`、`effort`。

---

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `joints` | `string[]` | `[]` | 受控关节名称列表，不能为空 |
| `control_mode` | `string` | `"position_ff"` | 控制模式：`position_ff` 或 `gravity_comp` |
| `urdf_path` | `string` | `""` | URDF 文件路径，仅 `gravity_comp` 模式需要 |

---

## 插件信息

- **命名空间**：`robot_controller`
- **类名**：`robot_controller::MotorController`
- **插件描述文件**：`motor_controller_plugin.xml`
- **基类**：`controller_interface::ControllerInterface`

---
## 参考资料

- [ROS2 Controllers 官方文档 - 编写自定义控制器](https://control.ros.org/jazzy/doc/ros2_controllers/doc/writing_new_controller.html)
- [ros2_controllers 源码](https://github.com/ros-controls/ros2_controllers)
- [达秒电机 MIT 控制模式文档](https://gl1po2nscb.feishu.cn/wiki/Y3OEwMr4GivZU9kZqkjctmGinye)
- [ros2插件编写](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Pluginlib.html#creating-and-using-plugins-c)
