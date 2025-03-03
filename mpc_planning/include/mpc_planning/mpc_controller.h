#ifndef MPC_PLANNING_MPC_CONTROLLER_H
#define MPC_PLANNING_MPC_CONTROLLER_H

#include <Eigen/Dense>
#include "mpc_planning/uav_model.h"
#include <geometry_msgs/Point.h>
namespace mpc_planning {

class MPCController {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MPCController(double dt = 0.1, int horizon = 10);

    void setWaypoint(const geometry_msgs::Point& waypoint);
    
    //公共接口
    Eigen::Vector2d solve(const Eigen::Vector2d& current_state);

private:
    // MPC 参数
    double dt_;              // 时间步长
    int horizon_;           // 预测时域
    
    // 权重
    double w1;     // 到waypoint的距离
    double w2;     // 输入
    
    // 系统矩阵
    Eigen::Matrix2d A_;     
    Eigen::Matrix2d B_;     
    Eigen::MatrixXd F_;   
    Eigen::MatrixXd Q_;

    Eigen::Vector2d waypoint_;
    // 求解二次规划问题
    Eigen::Vector2d solveQP(const Eigen::Vector2d& current_state);
};

} // namespace mpc_planning

#endif // MPC_PLANNING_MPC_CONTROLLER_H 