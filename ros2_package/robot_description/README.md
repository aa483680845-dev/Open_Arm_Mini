# robot_description

## 概述

本包包含六自由度机械臂的所有模型描述文件，包括 URDF、xacro 配-置、STL 网格模型，以及 Mujoco 仿真所需的 XML 文件。

---

## 包结构

```
robot_description/
├── urdf/
│   ├── robot_1.urdf                    # 主 URDF 文件（由 SW 插件导出）
│   ├── my_robot.xacro                  # xacro 入口文件
│   ├── robot_1.ros2_control.xacro      # ros2_control 硬件接口配置（含 MIT 增益）
│   └── Mujoco_xml/
│       ├── robot_mujoco.xml            # Mujoco 机器人模型
│       └── scene.xml                   # Mujoco 场景（地面、光照）
├── meshes/                             # 各连杆 STL 网格模型
│   ├── base_link.STL
│   ├── link_1.STL ~ link_6.STL
├── config/
│   └── default.rviz                    # RViz 默认配置
└── launch/
    └── robot_1_rviz.launch.py          # 启动 RViz 可视化
```

---

## 硬件接口配置（`robot_1.ros2_control.xacro`）

这是最常需要修改的文件，用于切换控制插件和调整 MIT 增益参数。

### 插件切换

修改 `<plugin>` 标签选择运行模式：

| 插件 | 用途 | 备注 |
|------|------|------|
| `mock_components/GenericSystem` | 模拟组件 | 默认，用于代码调试，无需硬件 |
| `mobile_base_hardware/MobileBaseHardwareInterface` | 真实硬件 | 需接 USB2CAN 连接电机 |
| `mujoco_ros2_control/MujocoSystemInterface` | Mujoco 仿真 | 需配合 `robot_mujoco.xml` 使用 |

```xml
<hardware>
    <!-- 三选一，取消对应行的注释 -->
    <plugin>mock_components/GenericSystem</plugin>
    <!-- <plugin>mobile_base_hardware/MobileBaseHardwareInterface</plugin> -->
    <!-- <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
    <param name="mujoco_model">$(find robot_description)/urdf/Mujoco_xml/robot_mujoco.xml</param> -->
</hardware>
```

### MIT 增益参数

各关节的 `kp`（位置增益）和 `kd`（速度增益）透传给达秒电机 MIT 阻抗控制器：

| 关节 | kp | kd | 对应电机 |
|------|----|----|----------|
| joint_1 | 80.0 | 10.0 | DM_4340 |
| joint_2 | 90.0 | 15.0 | DM_4340 |
| joint_3 | 120.0 | 20.0 | DM_4340 |
| joint_4 | 20.0 | 1.0 | DM_4310 |
| joint_5 | 20.0 | 1.0 | DM_4310 |
| joint_6 | 20.0 | 1.0 | DM_4310 |

> MIT 阻抗控制输出：`τ = kp*(q_cmd - q) + kd*(dq_cmd - dq) + τ_ff`
> 调整增益时请参考[达秒电机官方文档](https://gl1po2nscb.feishu.cn/wiki/Y3OEwMr4GivZU9kZqkjctmGinye)中各电机型号的参数上限。

---

## URDF 模型来源

机器人 URDF 由 **SolidWorks** 三维模型通过开源插件 [sw_urdf_exporter](https://wiki.ros.org/action/fullsearch/sw_urdf_exporter?action=fullsearch&context=180&value=linkto:%22sw_urdf_exporter%22) 导出，网格模型同步导出为 STL 格式。

详细操作教程可参考：[B站视频教程](https://www.bilibili.com/video/BV16izyYeEDm/?spm_id_from=333.1387.favlist.content.click&vd_source=06ddd9c4e5f9252a80487469164151bf)

---

## Mujoco 仿真

如果想要使用这个Mujoco仿真，一定要看下面这个开源项目，里面有详细的教程。
仿真基于开源项目 [mujoco_ros2_control](https://github.com/moveit/mujoco_ros2_control) 实现，包含两个 XML 文件：

- **`robot_mujoco.xml`**：机器人本体模型，包含连杆、关节、执行器定义
- **`scene.xml`**：仿真场景，包含地面平面和光照配置

启动仿真：

```bash
ros2 launch robot_bringup robot_mujoco.launch.py
```
