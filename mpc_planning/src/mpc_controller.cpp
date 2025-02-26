#include "mpc_planning/mpc_controller.h"

namespace mpc_planning {

MPCController::MPCController(double dt, int horizon)
    : dt_(dt)
    , horizon_(horizon)
{
    // 初始化权重矩阵
    Q_ = Eigen::MatrixXd::Identity(6, 6);
    Q_.topLeftCorner(2, 2) *= 100.0;  // 位置误差权重
    Q_.bottomRightCorner(2, 2) *= 10.0;  // 速度误差权重
    
    R_ = Eigen::MatrixXd::Identity(3, 3) * 1.0;  // 控制输入权重
    
    position_ref_ = Eigen::Vector2d::Zero();
}

void MPCController::setReference(const Eigen::Vector2d& position_ref) {
    position_ref_ = position_ref;
}

Eigen::Vector2d MPCController::solve(const Eigen::Vector2d& current_state) {
    return solveQP(current_state);
}

Eigen::Vector2d MPCController::solveQP(const Eigen::Vector2d& current_state) {
    // 这里需要实现二次规划求解器
    // 为了示例，我们先实现一个简单的 P 控制器
    Eigen::Vector2d position_error = position_ref_ - current_state.segment<2>(0);
    Eigen::Vector2d velocity_error = -current_state.segment<2>(0);  // 期望速度为0
    
    return position_error * 2.0 + velocity_error * 0.5;
}

} // namespace mpc_planning 