#include "my_robot_hardware/mobile_base_hardware_interface.hpp"
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

namespace
{
// 电机 CAN ID，与 dm_motor_add 中的 master_id 对应
constexpr uint8_t kMotorIds[mobile_base_hardware::kNumJoints] = {
  0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
}  // namespace

namespace mobile_base_hardware
{
  hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 从 URDF <param> 读取每个关节的 MIT 增益
    for (int i = 0; i < kNumJoints; i++)
    {
      const auto & params = info_.joints[i].parameters;
      auto kp_it = params.find("kp");
      auto kd_it = params.find("kd");
      if (kp_it == params.end() || kd_it == params.end())
      {
        RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"),
                     "joint_%d 缺少 kp 或 kd 参数，请检查 ros2_control xacro", i + 1);
        return hardware_interface::CallbackReturn::ERROR;
      }
      kp_[i] = std::stod(kp_it->second);
      kd_[i] = std::stod(kd_it->second);
      RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                  "joint_%d  kp=%.1f  kd=%.1f", i + 1, kp_[i], kd_[i]);
    }

    handle = damiao_handle_create(DEV_USB2CANFD_DUAL);

    int device_cnt = damiao_handle_find_devices(handle);
    if (device_cnt == 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("MobileBaseHardwareInterface"), "no device found!");
      damiao_handle_destroy(handle);
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state;
    int handle_cnt = 0;
    damiao_handle_get_devices(handle, dev_list, &handle_cnt);

    if (device_open(dev_list[0]))
    {
      RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "device opened!");
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("MobileBaseHardwareInterface"), "device open failed!");
      damiao_handle_destroy(handle);
      return hardware_interface::CallbackReturn::ERROR;
    }
    char strBuf[255] = {0};
    device_get_version(dev_list[0], strBuf, sizeof(strBuf));
    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "device version:%s", strBuf);
    device_get_serial_number(dev_list[0], strBuf, sizeof(strBuf));
    RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "device sn:%s", strBuf);
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state;
    uint32_t nom_baud = 1000000;
    uint32_t dat_baud = 5000000;

    /* 初始化通道波特率 */
    dm_init_channel(dev_list[0], 0, true, nom_baud, dat_baud, 0.75f, 0.75f);

    /* 添加电机 */
    dm_motor_add(DM_4340, 0x01, 0x11);
    dm_motor_add(DM_4340, 0x02, 0x12);
    dm_motor_add(DM_4340, 0x03, 0x13);
    dm_motor_add(DM_4310, 0x04, 0x14);
    dm_motor_add(DM_4310, 0x05, 0x15);
    dm_motor_add(DM_4310, 0x06, 0x16);
    /* 开启通道 */
    device_open_channel(dev_list[0], 0);
    /* 注册回调 */
    device_hook_to_rec(dev_list[0], dm_receive_callback);

    dm_control_enable(dev_list[0]);

    // 等待电机CAN回调至少收到一帧有效反馈，避免 state_q 为0时初始化 command
    // 最多等待500ms，每10ms轮询一次
    bool got_feedback = false;
    for (int retry = 0; retry < 50; retry++)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      // 只要任意一轴 state_q 非零就认为收到了反馈
      for (int i = 0; i < kNumJoints; i++)
      {
        double q, dq, tau;
        dm_motor_snapshot(i, &q, &dq, &tau);
        if (std::abs(q) > 1e-6)
        {
          got_feedback = true;
          break;
        }
      }
      if (got_feedback) break;
    }

    if (!got_feedback)
    {
      RCLCPP_WARN(rclcpp::get_logger("MobileBaseHardwareInterface"),
                  "未在500ms内收到电机反馈，初始命令可能为0，请确认电机已上电！");
    }

    for(int i = 0 ; i < kNumJoints ; i++)
    {
       double q, dq, tau;
       dm_motor_snapshot(i, &q, &dq, &tau);
       joint_last_state_pos[i] = q;  // 初始化滤波历史，避免上电跳零
       std::string joint = "joint_" + std::to_string(i + 1);
       set_command(joint + "/position", q);
       set_command(joint + "/velocity", 0.0);
       set_command(joint + "/effort",   0.0);
       RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"),
                   "joint_%d 初始命令: pos=%.4f rad, vel=0, eff=0", i + 1, q);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  hardware_interface::CallbackReturn MobileBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {
    (void)previous_state;
    dm_control_disable();
    return hardware_interface::CallbackReturn::SUCCESS;
  }


  hardware_interface::return_type MobileBaseHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    (void)time;
    (void)period;
    double alpha = 0.4;
  for(int i = 0 ; i < kNumJoints ; i++)
  {
    double q, dq, tau;
    dm_motor_snapshot(i, &q, &dq, &tau);

    double delta = fabs(q - joint_last_state_pos[i]);
    double filtered;
    if(delta > 0.2)
    {
      filtered = joint_last_state_pos[i];  // 尖峰拒绝：保持上一帧值
    }
    else
    {
      filtered = alpha * q + (1 - alpha) * joint_last_state_pos[i];  // 低通滤波
    }
    joint_last_state_pos[i] = filtered;  // 用滤波后的值更新历史
    joint_state_pos[i] = filtered;

    std::string joint = "joint_" + std::to_string(i + 1);
    set_state(joint + "/position", filtered);
    set_state(joint + "/velocity", dq);
  }

    return hardware_interface::return_type::OK;
  }
  hardware_interface::return_type MobileBaseHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    (void)time;
    (void)period;

    for (int i = 0; i < kNumJoints; i++)
    {
      std::string joint = "joint_" + std::to_string(i + 1);
      double cmd_pos = get_command(joint + "/position");
      double cmd_vel = get_command(joint + "/velocity");
      double cmd_eff = get_command(joint + "/effort");

      // NaN 保护：任意分量异常则保持当前状态，跳过本帧发送
      if (std::isnan(cmd_pos) || std::isnan(cmd_vel) || std::isnan(cmd_eff))
      {
        static int nan_count[kNumJoints] = {};
        if (++nan_count[i] >= 10)
        {
          RCLCPP_WARN(rclcpp::get_logger("MobileBaseHardwareInterface"),
                      "joint_%d 收到 NaN 命令，跳过本帧", i + 1);
          nan_count[i] = 0;
        }
        continue;
      }

      // 透传：pos / vel / eff 全部来自控制器，hardware 不做任何计算
      dm_control_mit(dev_list[0], kMotorIds[i], kp_[i], kd_[i], cmd_pos, cmd_vel, cmd_eff);
    }
    return hardware_interface::return_type::OK;
  }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)
