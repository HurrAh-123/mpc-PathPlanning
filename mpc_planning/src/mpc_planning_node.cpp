#include <ros/ros.h>
#include "mpc_planning/uav_model.h"
#include "mpc_planning/mpc_controller.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <memory>


class MpcPlanningNode {
public:
    MpcPlanningNode():nh_("~") {
       // 参数服务器获取时间步长和预测时域
       double dt;
       int horizon;
       nh_.param<double>("dt", dt, 0.1);  // 从参数服务器获取时间步长，默认0.1秒
       nh_.param<int>("horizon", horizon, 10);  // 从参数服务器获取预测时域，默认10

       // 初始化无人机模型及 MPC 控制器
       uav_model_ = std::make_shared<mpc_planning::UAVModel>(dt);
       mpc_controller_ = std::make_shared<mpc_planning::MPCController>(dt, horizon);
        
       // 订阅x,y坐标
       pose_sub_ = nh_.subscribe("uav/pose", 1, &MpcPlanningNode::poseCallback, this);

       // 发布xy方向速度
       vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        
       //mpc控制部分
       control_timer_ = nh_.createTimer(ros::Duration(dt),&MpcPlanningNode::controlCallback, this);
    }
private:
    void poseCallback(const geometry_msgs::Point::ConstPtr& msg) {
        // 只更新xy位置
        current_position_.x = msg->x;
        current_position_.y = msg->y;
        uav_model_->updateState(current_position_);
    }

    void controlCallback(const ros::TimerEvent&) {
        // 计算控制输入（速度）
        Eigen::Vector2d control = mpc_controller_->solve(uav_model_->getState());
            
        // 发布底盘速度指令
        geometry_msgs::Twist vvv;
        // vvv.linear.x = control(0);
        // vvv.linear.y = control(1);
        vvv.linear.x = 0.1;
        vvv.linear.y = 0.1;
        vvv.linear.z = 0.0;  
        vel_pub_.publish(vvv);
    }

    ros::NodeHandle nh_;
    std::shared_ptr<mpc_planning::UAVModel> uav_model_;//共享指针防止内存泄露
    std::shared_ptr<mpc_planning::MPCController> mpc_controller_;//共享指针防止内存泄露
    ros::Subscriber pose_sub_;//用来订阅当前位置（状态）
    ros::Publisher vel_pub_;//用来发布速度（系统输入）
    ros::Timer control_timer_;
    geometry_msgs::Point current_position_;  // 只存储当前xy位置
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mpc_planning_node");
    MpcPlanningNode node_1;
    
     // 定义问题维度
    int n = 2;  // 变量数量
    int m = 3;  // 约束数量

    
    ros::spin();
    return 0;
} 