#ifndef ROBOT_CONTROLLERS__MOTOR_CONTROLLER_HPP_
#define ROBOT_CONTROLLERS__MOTOR_CONTROLLER_HPP_

#include <memory>
#include <Eigen/Dense>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace robot_controller
{
using Float64MultiArray = std_msgs::msg::Float64MultiArray;

class MotorController : public controller_interface::ControllerInterface
{
public:
    MotorController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
    std::vector<std::string> joint_names_;
    std::string control_mode_;   // "position_ff" | "gravity_comp"

    rclcpp::Subscription<Float64MultiArray>::SharedPtr command_subscriber_; //  订阅指令
    realtime_tools::RealtimeBuffer<std::shared_ptr<Float64MultiArray>> rt_command_buffer_; // 实时缓冲区

    // position_ff 模式状态
    std::vector<double> joint_cmd_pos_;
    std::vector<double> joint_cmd_last_pos_;  // 上一帧滤波后指令（滤波历史 & dq_ff 基准）
    std::vector<double> joint_cmd_last_vel_;
    bool initialized_ = false;

    // gravity_comp 模式：Pinocchio 模型（on_configure 时按需构建）
    std::unique_ptr<pinocchio::Model> pin_model_;
    std::unique_ptr<pinocchio::Data>  pin_data_;
};

}  // namespace robot_controller

#endif  // ROBOT_CONTROLLERS__MOTOR_CONTROLLER_HPP_
