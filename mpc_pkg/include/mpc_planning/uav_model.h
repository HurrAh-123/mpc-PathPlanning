#ifndef MPC_PLANNING_UAV_MODEL_H
#define MPC_PLANNING_UAV_MODEL_H

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>

namespace mpc_planning {

class UAVModel {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW//内存对齐
    
    UAVModel(double dt = 0.1);
    
    void updateState(const geometry_msgs::Point& position);
    
    const Eigen::Vector2d& getState() const { return state_; }
    
private:
    double dt_;  // 时间步长
    Eigen::Vector2d state_;  // 简化的状态向量 [x, y]
};

} 

#endif

