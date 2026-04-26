# Open Arm Mini

基于 ROS2 的开源六自由度机械臂，通过 FDCAN 总线驱动达秒（Damiao）系列电机，采用 MIT 阻抗控制模式，支持真实硬件控制与 Mujoco 仿真，具备手柄末端控制和重力补偿功能。

另一个开源项目：[moveit2轨迹规划]

运行环境：Ubuntu 24.04 LTS + ROS2 Jazzy

## 演示

**手柄控制**

https://github.com/user-attachments/assets/Handle_control.mp4

**重力补偿**

https://github.com/user-attachments/assets/Gravity_compensation.mp4

## 目录结构

```
Open_Arm_Mini/
├── image/                  # 演示视频
├── SW_model/               # SolidWorks 机械臂三维模型
│   └── First_edition/
│       ├── SLDPRT/         # SolidWorks 原始零件文件
│       └── STEP/           # 通用 STEP 格式，可导入其他 CAD 软件
└── ros2_package/           # ROS2 控制软件包
    ├── robot_description/  # URDF/xacro 描述文件及 STL 网格模型
    ├── robot_dm_driver/    # 达秒电机底层驱动（FDCAN/MIT 协议）
    ├── my_robot_hardware/  # ROS2 硬件接口层
    ├── robot_controllers/  # 电机控制器（手柄控制 / 重力补偿）
    ├── robot_pinocchio/    # Pinocchio 逆运动学节点
    ├── robot_bringup/      # Launch 启动文件及控制器配置
    └── README.md           # 软件包详细文档
```

## 软件包文档

ROS2 软件包的详细说明（环境配置、编译运行、架构设计、各包接口）请参阅：

👉 [ros2_package/README.md](ros2_package/README.md)
