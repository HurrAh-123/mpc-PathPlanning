#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
using namespace Eigen;

Vector2d start_pos;
Vector2d waypoint_pos;
bool waypoint_set = false;

ros::Publisher global_path_pub;
ros::Subscriber waypoint_sub;
ros::Subscriber start_sub;

void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  waypoint_pos.x() = msg->pose.position.x;
  waypoint_pos.y() = msg->pose.position.y;
  waypoint_set = true;
}

void startCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  start_pos.x() = msg->pose.pose.position.x;
  start_pos.y() = msg->pose.pose.position.y;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_path_publisher");
  ros::NodeHandle nh("~");
  
  start_sub = nh.subscribe("/odom", 1, startCallback);
  waypoint_sub= nh.subscribe("/move_base_simple/goal", 1, waypointCallback);
  global_path_pub = nh.advertise<nav_msgs::Path>("/global_path", 1);
  
  const double step = 0.1;

  ros::Rate rate(10);
  while (ros::ok())
  {
    if (!waypoint_set) {
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    double dist = (waypoint_pos - start_pos).norm();//计算向量距离
    Vector2d diff = (waypoint_pos - start_pos) / dist;//计算单位向量

    nav_msgs::Path global_path;
    global_path.header.stamp = ros::Time::now();
    global_path.header.frame_id = "world";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    pose_stamped.pose.orientation.w = 1;

    int idx = 0;
    for (double i = 0.0; i < dist; i += step)
    {
      pose_stamped.header.seq = idx++;

      Vector2d waypoint = start_pos + i * diff;
      pose_stamped.pose.position.x = waypoint.x();
      pose_stamped.pose.position.y = waypoint.y();
      pose_stamped.pose.position.z = 0;

      global_path.poses.push_back(pose_stamped);
    }

    waypoint_set = false;
    ROS_INFO("start_pos: %f, %f", start_pos.x(), start_pos.y());
    ROS_INFO("waypoint_pos: %f, %f", waypoint_pos.x(), waypoint_pos.y());
    global_path_pub.publish(global_path);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}