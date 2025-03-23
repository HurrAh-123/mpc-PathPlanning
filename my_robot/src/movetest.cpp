#include <ros/ros.h>
#include <chrono>
#include <std_msgs/Bool.h>

#include "moving_cylinder.hpp"

using namespace std;

const uint8_t cylinder_num = 5;// 定义5个圆柱体
int MovingCylinder::id_ = 0;// 圆柱体ID初始化

bool is_move = false;

void cmdCallback(const std_msgs::Bool::ConstPtr& cmd_move)
{
  is_move = cmd_move->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movetest");
  ros::NodeHandle n("~");

  ros::Subscriber sub = n.subscribe<std_msgs::Bool>("/cmd_move", 10, cmdCallback);// 订阅cmd_move话题
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState set_model_state_srv;

  // 初始化圆柱体 
  MovingCylinder cylinder[cylinder_num];
  geometry_msgs::Point init_pos[cylinder_num];// 初始位置
  geometry_msgs::Twist twist[cylinder_num];// 速度

  // 初始化圆柱体的xyz和速度vx、vy
  for (uint8_t i = 0; i < cylinder_num; i++)
  {
    n.param<double>("x" + to_string(i), init_pos[i].x, 5);
    n.param<double>("y" + to_string(i), init_pos[i].y, 5);
    n.param<double>("z" + to_string(i), init_pos[i].z, 0.25);
    n.param<double>("vx" + to_string(i), twist[i].linear.x, 0.0);
    n.param<double>("vy" + to_string(i), twist[i].linear.y, 0.0);

    cylinder[i].setPosition(init_pos[i]);// 设置初始位置
    cylinder[i].setVel(twist[i]);// 设置速度
  }

  while (ros::ok())
  {
    if (is_move)// 如果收到运动命令
    {
      // 更新所有圆柱体的状态
      for (uint8_t i = 0; i < cylinder_num; i++)
        cylinder[i].updateState();
    }

    for (uint8_t i = 0; i < cylinder_num; i++)
    {
      set_model_state_srv.request.model_state = cylinder[i].model_state_;
      client.call(set_model_state_srv);// 调用服务来更新圆柱体的状态
    }

    ros::spinOnce();
    ros::Duration(0.01).sleep();
    
  }
  return 0;
}
