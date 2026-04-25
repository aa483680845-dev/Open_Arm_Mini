#include "robot_pinocchio/pinocchio_inverse.hpp"
#include <cmath>
pinocchio_inverse::pinocchio_inverse()
{
  J_.resize(6, 6);
  J_.setZero();
  v_.resize(6);
  v_.setZero();
}

pinocchio_inverse::~pinocchio_inverse() {}

void pinocchio_inverse::update(pinocchio::Model &model, pinocchio::Data &data,
                                const Eigen::VectorXd &q, const JoyData &joy)
{
  // 用实际关节角做正运动学，获取末端当前位姿
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateGlobalPlacements(model, data);
  v_change_ = joy.axes[DPAD_Y];   
  if (v_change_ > 0.5 && v_change_last < 0.5)  { factor += 0.01; }
  if (v_change_ < -0.5 && v_change_last > -0.5) { factor -= 0.01; }
  v_change_last = v_change_;
  factor = std::clamp(factor,0.01,0.05);
  
    // 平移
    v_world(0) = joy.axes[LEFTY] * factor;
    v_world(1) = joy.axes[LEFTX] * factor;
    v_world(2) = joy.axes[RIGHTY] * factor;
    // 姿态
    v_world(3) = 0.0;
    v_world(4) = 0.0;
    v_world(5) = 0.0;

    ik_solve(model, data, v_world, q);
}

void pinocchio_inverse::ik_solve(pinocchio::Model &model, pinocchio::Data &data,
                                  const Eigen::Matrix<double, 6, 1> &v_des,const Eigen::VectorXd &q)
{
  // 用 IK 当前解做正运动学，与雅可比保持一致
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateGlobalPlacements(model, data);

  const pinocchio::SE3 &oMi = data.oMi[JOINT_ID];
  pinocchio::Motion v_world_motion(v_des.head<3>(), v_des.tail<3>());
  pinocchio::Motion v_local_motion = oMi.actInv(v_world_motion);
  Vector6d v_local = v_local_motion.toVector();

  // 计算雅可比并修正（含 Jlog6 修正项）
  pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J_);
  JJt_.noalias() = J_ * J_.transpose();
  double damp = 1e-4;
  JJt_.diagonal().array() += damp;
  // 求关节速度
  v_.noalias() = J_.transpose() * JJt_.ldlt().solve(v_local);

}
