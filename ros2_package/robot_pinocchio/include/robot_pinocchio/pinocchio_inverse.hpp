#ifndef PINOCCHIO_INVERSE_HPP
#define PINOCCHIO_INVERSE_HPP
#include <array>
#include <algorithm>
#include <Eigen/Dense>
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/algorithm/rnea.hpp>
enum AxisIndex
{
  LEFTX = 0,
  LEFTY = 1,
  LEFT_TRIGGER = 2,
  RIGHTX = 3,
  RIGHTY = 4,
  RIGHT_TRIGGER = 5,
  DPAD_X = 6,
  DPAD_Y = 7
};

enum ButtonIndex
{
  BUTTON_A = 0,
  BUTTON_B = 1,
  BUTTON_X = 2,
  BUTTON_Y = 3,
  BUTTON_LB = 4,
  BUTTON_RB = 5
};

struct JointStateData {
  std::array<double, 6> positions = {};
};

struct JoyData {
  std::array<double, 8> axes   = {};
  std::array<int, 6>    buttons = {};
};

class pinocchio_inverse {
public:
  pinocchio_inverse();
  ~pinocchio_inverse();

  // 每个控制周期调用一次，输入实际关节角和手柄数据，结果写入 qik
  void update(pinocchio::Model &model, pinocchio::Data &data,
              const Eigen::VectorXd &q, const JoyData &joy);
  //输出计算出的关节速度
  Eigen::VectorXd v_;
  // 奇异度指标：<1 接近奇异，>=1 正常
  double ratio = 1.0;
  Eigen::Vector3d last_sync_trans_ = Eigen::Vector3d::Zero();
  double factor = 0.03;               // 速度关节变量 
private:
  static constexpr int    JOINT_ID = 6;
  static constexpr double IK_EPS   = 0.01;  // 奇异点判断阈值
  static constexpr double IK_DT    = 0.04;   // IK 积分步长

  // DLS 逆运动学求解，结果更新至 qik
  void ik_solve(pinocchio::Model &model, pinocchio::Data &data,
                const Eigen::Matrix<double, 6, 1> &v_des,const Eigen::VectorXd &q);

  // 手柄累加状态
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d v_world = Vector6d::Zero();

  // 模式标志
  bool initialized_  = false;
  double v_change_ = 0;
  double v_change_last = 0;
  // IK 中间量（预分配，避免频繁动态内存）
  Eigen::MatrixXd          J_;
  pinocchio::Data::Matrix6 JJt_;
};

#endif // PINOCCHIO_INVERSE_HPP
