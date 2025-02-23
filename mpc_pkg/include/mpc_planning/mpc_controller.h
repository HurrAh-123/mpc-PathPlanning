#ifndef MPC_PLANNING_MPC_CONTROLLER_H
#define MPC_PLANNING_MPC_CONTROLLER_H

#include <Eigen/Dense>
#include "mpc_planning/uav_model.h"

namespace mpc_planning {

class MPCController {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MPCController(double dt = 0.1, int horizon = 10);

    // 设置参考轨迹点
    void setReference(const Eigen::Vector2d& position_ref);

    // 计算控制输入
    Eigen::Vector2d solve(const Eigen::Vector2d& current_state);

private:
    // MPC 参数
    double dt_;              // 时间步长
    int horizon_;           // 预测时域
    
    // 权重矩阵
    Eigen::Matrix2d Q_;     // 状态误差权重
    Eigen::Matrix2d R_;     // 控制输入权重
    
    // 参考轨迹
    Eigen::Vector2d position_ref_;

    // 求解二次规划问题
    Eigen::Vector2d solveQP(const Eigen::Vector2d& current_state);
};

} // namespace mpc_planning

#endif // MPC_PLANNING_MPC_CONTROLLER_H 