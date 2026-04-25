#ifndef MOBILE_BASE_HARDWARE_INTERFACE_HPP
#define MOBILE_BASE_HARDWARE_INTERFACE_HPP

#include <cstdint>
#include "hardware_interface/system_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "robot_dm_driver/dm_motor.hpp"
namespace mobile_base_hardware
{
     constexpr int kNumJoints = 6;

     class MobileBaseHardwareInterface : public hardware_interface::SystemInterface
     {
     public:
          // 声明周期节点覆盖

          hardware_interface::CallbackReturn
          on_configure(const rclcpp_lifecycle::State &previous_state) override;
          hardware_interface::CallbackReturn
          on_activate(const rclcpp_lifecycle::State &previous_state) override;
          hardware_interface::CallbackReturn
          on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
          
          // 系统接口覆盖
          hardware_interface::CallbackReturn
          on_init(const hardware_interface::HardwareInfo &info) override;
          hardware_interface::return_type
          read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
          hardware_interface::return_type
          write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

     private:
          // MIT增益 —— 从URDF <param> 读取，透传给 dm_control_mit
          double kp_[kNumJoints];
          double kd_[kNumJoints];

          // 状态滤波
          double joint_state_pos[kNumJoints];
          double joint_last_state_pos[kNumJoints];

          damiao_handle *handle;
          device_handle *dev_list[2];
     };

}

#endif