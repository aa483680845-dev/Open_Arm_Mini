#include "robot_controllers/motor_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace robot_controller
{

MotorController::MotorController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn MotorController::on_init()
{
    joint_names_  = auto_declare<std::vector<std::string>>("joints", {}); 
    control_mode_ = auto_declare<std::string>("control_mode", "position_ff");
    auto_declare<std::string>("urdf_path", "");
    return controller_interface::CallbackReturn::SUCCESS; //初始化声明参数
}

controller_interface::CallbackReturn MotorController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    // 获取配置文件写入的参数
    joint_names_  = get_node()->get_parameter("joints").as_string_array();
    control_mode_ = get_node()->get_parameter("control_mode").as_string(); 

    if (joint_names_.empty())
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must not be empty.");
        return controller_interface::CallbackReturn::ERROR;
    }

    if (control_mode_ != "position_ff" && control_mode_ != "gravity_comp")
    {
        RCLCPP_ERROR(get_node()->get_logger(),
            "Unknown control_mode '%s'. Use 'position_ff' or 'gravity_comp'.",
            control_mode_.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    if (control_mode_ == "gravity_comp")
    {   //如果是重力补偿控制模式，需要为pinocchio模型获取urdf文件路径
        const std::string urdf_path =
            get_node()->get_parameter("urdf_path").as_string();

        if (urdf_path.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                "gravity_comp mode requires 'urdf_path' parameter.");
            return controller_interface::CallbackReturn::ERROR;
        }

        try
        {
            pin_model_ = std::make_unique<pinocchio::Model>();
            pinocchio::urdf::buildModel(urdf_path, *pin_model_);
            pin_data_  = std::make_unique<pinocchio::Data>(*pin_model_);
            RCLCPP_INFO(get_node()->get_logger(),
                "Pinocchio model loaded: nq=%d  nv=%d  urdf=%s",
                pin_model_->nq, pin_model_->nv, urdf_path.c_str());
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                "Failed to build Pinocchio model: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    auto callback = [this](const Float64MultiArray::SharedPtr msg) -> void {
        rt_command_buffer_.writeFromNonRT(msg);  
    };

    command_subscriber_ = get_node()->create_subscription<Float64MultiArray>(
        "arm_MIT_command_controller/commands", rclcpp::SystemDefaultsQoS(), callback); //定义订阅者，接收上游节点数据

    RCLCPP_INFO(get_node()->get_logger(),
        "MotorController configured: %zu joints, mode='%s'.",
        joint_names_.size(), control_mode_.c_str());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MotorController::command_interface_configuration() const //配置命令接口
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & joint_name : joint_names_)
    {
        config.names.push_back(joint_name + "/position");
        config.names.push_back(joint_name + "/velocity");
        config.names.push_back(joint_name + "/effort");
    }
    return config;
}

controller_interface::InterfaceConfiguration
MotorController::state_interface_configuration() const //配置状态接口
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto & joint_name : joint_names_)
    {
        config.names.push_back(joint_name + "/position");
        config.names.push_back(joint_name + "/velocity");
        config.names.push_back(joint_name + "/effort");
    }
    return config;
}

controller_interface::CallbackReturn MotorController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;

    // Each joint claims 3 command interfaces: position / velocity / effort
    const size_t expected = joint_names_.size() * 3;
    if (command_interfaces_.size() != expected)
    {
        RCLCPP_ERROR(
            get_node()->get_logger(),
            "Expected %zu command interfaces, got %zu.",
            expected, command_interfaces_.size());
        return controller_interface::CallbackReturn::ERROR;
    }
    
    // 初始化 command = current state
    initialized_ = false;
    joint_cmd_pos_.resize(joint_names_.size(), 0.0);
    joint_cmd_last_pos_.resize(joint_names_.size(), 0.0);
    joint_cmd_last_vel_.resize(joint_names_.size(), 0.0);

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        double pos = state_interfaces_[i * 3 + 0].get_value(); //获取状态接口数据，状态接口数据接收的是hardware数据
        double vel = state_interfaces_[i * 3 + 1].get_value();
        double eff = 0.f;

        joint_cmd_pos_[i]      = pos;
        joint_cmd_last_pos_[i] = pos;
        joint_cmd_last_vel_[i] = vel;

        command_interfaces_[i * 3 + 0].set_value(pos);  
        command_interfaces_[i * 3 + 1].set_value(vel);
        command_interfaces_[i * 3 + 2].set_value(eff);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MotorController::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MotorController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period) //开始循环处理数据
{
    (void)time;

    if (control_mode_ == "position_ff")
    {
        auto cmd_msg = rt_command_buffer_.readFromRT(); //读取缓冲区数据
        if (!cmd_msg || !(*cmd_msg))
        {
            return controller_interface::return_type::OK;
        }
        const auto & data = (*cmd_msg)->data;

        // dt 限幅：防止首帧或调度抖动导致 dq_ff 飞速
       double dt = std::clamp(period.seconds(), 0.0005, 0.01);
        constexpr double kFilterTau = 0.005;
        const double alpha = 0.4;

        if (!initialized_)
        {
            for (size_t i = 0; i < joint_names_.size(); ++i)
                {
                joint_cmd_last_pos_[i] = state_interfaces_[i * 3 + 0].get_value();
                joint_cmd_last_vel_[i] = state_interfaces_[i * 3 + 1].get_value();}
            initialized_ = true;
        }

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            const double raw  = data[i * 3 + 1];
            const double last = joint_cmd_last_vel_[i];

            // 使用低通滤波
            double filtered = last + alpha * (raw - last);

            joint_cmd_pos_[i] = joint_cmd_last_pos_[i] + filtered * dt;
            joint_cmd_last_vel_[i] = filtered;
            joint_cmd_last_pos_[i] = joint_cmd_pos_[i];
            
            command_interfaces_[i * 3 + 0].set_value(joint_cmd_pos_[i]);  // pos
            command_interfaces_[i * 3 + 1].set_value(filtered);     // vel ff
            command_interfaces_[i * 3 + 2].set_value(0.0);       // no torque ff
        }


    }
    else  // gravity_comp
    {
        const size_t n = joint_names_.size();

        // 从 state_interfaces 读当前关节角（每关节3个：pos/vel/effort）
        Eigen::VectorXd q(static_cast<Eigen::Index>(n));
        for (size_t i = 0; i < n; ++i)
            q[static_cast<Eigen::Index>(i)] = state_interfaces_[i * 3 + 0].get_value();

        // Pinocchio RNEA：计算重力补偿力矩 tau = g(q)
        // computeGeneralizedGravity 等价于 rnea(q, 0, 0)，结果存入 pin_data_->g
        pinocchio::computeGeneralizedGravity(*pin_model_, *pin_data_, q);
        const Eigen::VectorXd & tau_gravity = pin_data_->g;

        // 发送给 hardware：
        //   pos = q_now  →  kp*(q_now - q_actual) ≈ 0，位置增益项几乎不出力
        //   vel = 0      →  kd*(0 - dq_actual) = -kd*dq，提供速度阻尼
        //   eff = tau_g  →  直接叠加重力补偿力矩
        for (size_t i = 0; i < n; ++i)
        {
            command_interfaces_[i * 3 + 0].set_value(q[static_cast<Eigen::Index>(i)]);
            command_interfaces_[i * 3 + 1].set_value(0.0);
            command_interfaces_[i * 3 + 2].set_value(tau_gravity[static_cast<Eigen::Index>(i)]);
        }
    }

    return controller_interface::return_type::OK;
}

}  // namespace robot_controller

PLUGINLIB_EXPORT_CLASS(
    robot_controller::MotorController,
    controller_interface::ControllerInterface)
