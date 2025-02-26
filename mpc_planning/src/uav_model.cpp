#include "mpc_planning/uav_model.h"

namespace mpc_planning {

UAVModel::UAVModel(double dt) : dt_(dt) {
    state_ = Eigen::Vector2d::Zero(2);
}

void UAVModel::updateState(const geometry_msgs::Point& position) {
    state_[0] = position.x;
    state_[1] = position.y;
}
} // namespace mpc_planning 