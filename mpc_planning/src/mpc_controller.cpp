#include "mpc_planning/mpc_controller.h"
#include <qpOASES.hpp>
namespace mpc_planning {

MPCController::MPCController(double dt, int horizon)
    : dt_(dt)
    , horizon_(horizon)
    , w1(0.5)
    , w2(0.5)
{
    A_ << 1, 0,
          0, 1;
    B_ << dt_, 0,
          0, dt_;

    F_.resize(2 * horizon_, 2);
    F_.setZero();
    Eigen::Matrix2d temp_A = A_;
    for (int i = 0; i < horizon_; ++i) {
        F_.block<2, 2>(2 * i, 0) = temp_A;
        temp_A = temp_A * A_;  
    }

    Q_.resize(2 * horizon_, 2 * horizon_);
    Q_.setZero();
    for (int i = 0; i < horizon_; ++i) {
        temp_A = A_;
        for (int j = 0; j <= i; ++j) {
            Q_.block<2, 2>(2 * i, 2 * j) = temp_A * B_;
            temp_A = temp_A * A_;
        }
    }
    
    waypoint_= Eigen::Vector2d::Zero();//初始值先设置为0
}

void MPCController::setWaypoint(const geometry_msgs::Point& waypoint) {
    waypoint_ = Eigen::Vector2d(waypoint.x, waypoint.y);
}

Eigen::Vector2d MPCController::solve(const Eigen::Vector2d& current_state) {
    return solveQP(current_state);
}

Eigen::Vector2d MPCController::solveQP(const Eigen::Vector2d& current_state) {
    
    
    // H = w1 * Q^T * Q + w2
    Eigen::MatrixXd H = w1 * Q_.transpose() * Q_ + 
                        w2 * Eigen::MatrixXd::Identity(2*horizon_,2*horizon_);

    //g = w1 * (x^T * F^T * Q - P^T * Q)
    Eigen::VectorXd P = Eigen::VectorXd::Zero(2 * horizon_);
        for(int i = 0; i < horizon_; ++i) {
            P.segment<2>(2*i) = waypoint_;
        }
    Eigen::Vector2d x = current_state;
    Eigen::VectorXd g = w1 * 
    (x.transpose() * F_.transpose() * Q_ - P.transpose() * Q_);

    //约束（没看过记得改）

    double v_max = 1.0;  // 最大速度限制
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(2, -v_max);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(2, v_max);
    
    // 配置qpOASES
    qpOASES::QProblem problem(2, 0);
    qpOASES::Options options;
    options.printLevel = qpOASES::PL_NONE;
    problem.setOptions(options);
    
    // 求解QP
    int nWSR = 1000;
    problem.init(H.data(), g.data(), nullptr, lb.data(), ub.data(), 
                nullptr, nullptr, nWSR);
    
    // 获取结果
    Eigen::VectorXd solution(2);
    problem.getPrimalSolution(solution.data());
    
    // 返回第一个控制输入
    return solution.head<2>();
}

} // namespace mpc_planning 