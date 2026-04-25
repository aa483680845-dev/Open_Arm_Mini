#include "robot_pinocchio/pinocchio_inverse.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/callback_group.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
using Float64MultiArray = std_msgs::msg::Float64MultiArray;
using JointState = sensor_msgs::msg::JointState;
using Joy = sensor_msgs::msg::Joy;
using namespace std::placeholders;

class PinocchioInverseNode : public rclcpp::Node
{
public:
  PinocchioInverseNode() : Node("pinocchio_inverse_node")
  {
    callback_group_joint_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_joy_   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    std::string urdf_path = ament_index_cpp::get_package_share_directory("robot_description")
                            + "/urdf/robot_1.urdf";
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);

    auto joint_options = rclcpp::SubscriptionOptions();
    joint_options.callback_group = callback_group_joint_;
    joint_state_sub_ = this->create_subscription<JointState>(
        "joint_states", 10,
        std::bind(&PinocchioInverseNode::jointStateCallback, this, _1),
        joint_options);

    auto joy_options = rclcpp::SubscriptionOptions();
    joy_options.callback_group = callback_group_joy_;
    joy_sub_ = this->create_subscription<Joy>(
        "joy", 10,
        std::bind(&PinocchioInverseNode::joyCallback, this, _1),
        joy_options);

    joint_command_pub_ = this->create_publisher<Float64MultiArray>(
        "arm_MIT_command_controller/commands", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(4),
        std::bind(&PinocchioInverseNode::timerCallback, this),
        callback_group_timer_);

    RCLCPP_INFO(this->get_logger(), "PinocchioInverseNode initialized. nq=%d, nv=%d",
                model_.nq, model_.nv);
  }

private:
  void jointStateCallback(const JointState::SharedPtr msg)
  {
    if (msg->position.size() < 6) return;
    std::lock_guard<std::mutex> lock(joint_mutex_);
    for (int i = 0; i < 6; ++i)
      joint_data_.positions[i] = msg->position[i];
    joint_state_received_ = true;
  }

  void joyCallback(const Joy::SharedPtr msg)
  {
    if (msg->axes.size() < 8 || msg->buttons.size() < 6) return;
    std::lock_guard<std::mutex> lock(joy_mutex_);
    for (size_t i = 0; i < 8; ++i)
      joy_data_.axes[i] = msg->axes[i];
    for (size_t i = 0; i < 6; ++i)
      joy_data_.buttons[i] = msg->buttons[i];
  }

  void timerCallback()
  {
    // 在收到第一帧关节状态前不发布任何指令，避免跳变到零
    if (!joint_state_received_) return;

    // 1. 无锁快照
    JointStateData joint_snap;
    JoyData        joy_snap;
    
    { std::lock_guard<std::mutex> lock(joint_mutex_);
      joint_snap = joint_data_; }
    { std::lock_guard<std::mutex> lock(joy_mutex_);
      joy_snap   = joy_data_;   }

    // 2. 组装关节角向量
    Eigen::VectorXd q(6);
    for (size_t i = 0; i < 6; ++i)
      q[i] = joint_snap.positions[i];

    // 3. 运动控制库处理（手柄累加 + IK 求解）
    ctrl_.update(model_, data_, q, joy_snap);

    // 4. 发布关节指令
    Float64MultiArray cmd_msg;
    cmd_msg.data.resize(18);
    for (size_t i = 0; i < 6; ++i) {
      cmd_msg.data[i * 3 + 0] = 0.0;
      cmd_msg.data[i * 3 + 1] = ctrl_.v_[i];
      cmd_msg.data[i * 3 + 2] = 0.0;
    }
    joint_command_pub_->publish(cmd_msg);
  }

  // ROS2 通信
  rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<Joy>::SharedPtr        joy_sub_;
  rclcpp::Publisher<Float64MultiArray>::SharedPtr joint_command_pub_;
  rclcpp::TimerBase::SharedPtr                timer_;
  rclcpp::CallbackGroup::SharedPtr            callback_group_joint_;
  rclcpp::CallbackGroup::SharedPtr            callback_group_joy_;
  rclcpp::CallbackGroup::SharedPtr            callback_group_timer_;

  // 传感器数据缓冲
  JointStateData joint_data_;
  std::mutex     joint_mutex_;
  std::atomic<bool> joint_state_received_{false};
  JoyData        joy_data_;
  std::mutex     joy_mutex_;

  // 机器人模型
  pinocchio::Model model_;
  pinocchio::Data  data_;

  // 运动控制库
  pinocchio_inverse ctrl_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node    = std::make_shared<PinocchioInverseNode>();
  auto execute = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  execute->add_node(node);
  execute->spin();
  rclcpp::shutdown();
  return 0;
}
