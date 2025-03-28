#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>

#include "obs_kalman/kalman.hpp"

using namespace std;
using namespace Eigen;

ros::Subscriber _obs_sub;
ros::Publisher _obs_pub, _obs_vis_pub;

const size_t obs_kf_buffer_size = 100;

struct Ellipse
{
  double semimajor;  // 长轴
  double semiminor;  // 短轴
  double cx;         // 中心x坐标
  double cy;         // 中心y坐标
  double theta;      // 旋转角度
};

struct obs_param
{
  float x = 0.0;
  float y = 0.0;
  float a = 0.0;
  float b = 0.0;
  float theta = 0.0;
  float mea_cov;
};

class obs_kf
{
public:
  int N;
  float T;

  Kalman ka;//卡尔曼滤波器这一整个类
  std::vector<obs_param> param_list;
  std::vector<obs_param> pred_list;
  Eigen::VectorXd x; //未定义大小的向量
  Eigen::MatrixXd P0;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd H;
  Eigen::VectorXd z0;
  Eigen::VectorXd u_v;

  void obs_predict();
  obs_kf();
} _obs_kf[obs_kf_buffer_size];

obs_kf::obs_kf() : N(25)
{
  ka.m_StateSize = 9;// 状态向量维度：[x,y,a,b,θ,vx,vy,ax,ay]
  ka.m_MeaSize = 5;// 测量向量维度：[x,y,a,b,θ]
  ka.m_USize = 9;// 控制输入维度

  T = 0.1;

  x.resize(9);//1*9的矩阵
  x.setZero();

  //初始状态协方差
  float P0_cov = 0.025;
  P0.resize(9, 9);//9*9的矩阵
  P0.setIdentity();//单位矩阵
  P0 *= P0_cov;//P0_cov*单位矩阵

  //过程噪声协方差
  float Q_cov = 0.00008;
  Q.resize(9, 9);
  Q.setIdentity();
  Q *= Q_cov;

  //测量噪声协方差
  // R = [R_cov    0        0           0           0          ]  // x位置噪声，中等信任度
  //   [0        R_cov    0           0           0          ]  // y位置噪声，中等信任度
  //   [0        0        10*R_cov    0           0          ]  // a(长轴)噪声，较低信任度（10倍噪声）
  //   [0        0        0           10*R_cov    0          ]  // b(短轴)噪声，较低信任度（10倍噪声）
  //   [0        0        0           0           1e-8*R_cov ]  // θ(角度)噪声，很高信任度（很小的噪声）
  float R_cov = 0.0008;  //仿真 0.00005  实物  0.000003
  R.resize(5, 5);
  R.setZero();

  R(0, 0) = R_cov;
  R(1, 1) = R_cov;
  R(2, 2) = 10 * R_cov;
  R(3, 3) = 10 * R_cov;
  R(4, 4) = 0.00000001 * R_cov;

  A.resize(9, 9);
  A.setIdentity();
  A(0, 5) = A(1, 6) = T;

  B.resize(9, 9);
  B.setZero();

  H.resize(5, 9);
  H.setIdentity();

  z0.resize(5);
  z0.setZero();

  u_v.resize(9);
  u_v.setZero();

  ka.Init_Par(x, P0, R, Q, A, B, H, u_v);
}

void obs_kf::obs_predict()
{
  int num = param_list.size();
  if (num == 0)
    return;

  if (num == 1)
  {
    ka.m_x << param_list[0].x, param_list[0].y, param_list[0].a, param_list[0].b, param_list[0].theta, 0, 0, 0, 0;
    ka.m_P = P0;
  }
  else
  {
    float _mea_cov_min = 0.005;
    float _mea_cov_max = 0.05;
    float _k_cov_min = 1;
    float _k_cov_max = 3000;
    float _mea_cov = min(max(param_list.back().mea_cov, _mea_cov_min), _mea_cov_max);
    float _k = log10(_mea_cov / _mea_cov_min) / log10(_mea_cov_max / _mea_cov_min);
    float _k_cov = pow(_k_cov_max, _k) * pow(_k_cov_min, 1 - _k);

    ka.m_R(0, 0) = _k_cov * R(0, 0);
    ka.m_R(1, 1) = _k_cov * R(1, 1);

    VectorXd z(5);
    z << param_list.back().x, param_list.back().y, param_list.back().a, param_list.back().b, param_list.back().theta;

    ka.Predict_State();
    ka.Predict_Cov();
    
    ka.Cal_Gain();
    ka.Update_State();
    ka.Update_Cov();

    Kalman ka_tmp = ka;

    pred_list.clear();
    for (int i = 0; i < N; i++)
    {
      ka_tmp.Predict_State();
      ka_tmp.Predict_Cov();
      obs_param pred;
      float x_conf = 0.95 * sqrt(ka_tmp.m_P(0, 0));
      float y_conf = 0.95 * sqrt(ka_tmp.m_P(1, 1));
      float a_conf = 0.95 * sqrt(ka_tmp.m_P(2, 2));
      float b_conf = 0.95 * sqrt(ka_tmp.m_P(3, 3));
      float pos_conf = max(x_conf, y_conf);
      float ab_conf = max(a_conf, b_conf);

      pred.x = ka_tmp.m_x(0);
      pred.y = ka_tmp.m_x(1);
      pred.a = ka_tmp.m_x(2) + pos_conf + ab_conf;
      pred.b = ka_tmp.m_x(3) + pos_conf + ab_conf;
      pred.theta = ka.m_x(4);

      pred_list.push_back(pred);
    }
  }
}

//在回调函数里被调用的函数
void visEllipse(const vector<Ellipse>& obs_ellipses)
{
  visualization_msgs::MarkerArray ellipse_vis;

  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.ns = "ellipse";
  mk.type = visualization_msgs::Marker::CYLINDER;
  mk.action = visualization_msgs::Marker::ADD;
  mk.lifetime = ros::Duration(0.5);

  for (size_t i = 0; i < obs_ellipses.size(); i++)
  {
    mk.color.a = 0.4;
    mk.color.r = 0;
    mk.color.g = 1;
    mk.color.b = 0;

    mk.id = i;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = sin(obs_ellipses[i].theta / 2);
    mk.pose.orientation.w = cos(obs_ellipses[i].theta / 2);

    mk.pose.position.x = obs_ellipses[i].cx;
    mk.pose.position.y = obs_ellipses[i].cy;
    mk.pose.position.z = -0.3;

    mk.scale.x = 2 * obs_ellipses[i].semimajor;
    mk.scale.y = 2 * obs_ellipses[i].semiminor;
    mk.scale.z = 0.7;

    ellipse_vis.markers.push_back(mk);
  }

  _obs_vis_pub.publish(ellipse_vis);
}

void curve_fitting(obs_kf& obs, std_msgs::Float32MultiArray& obs_pub, vector<Ellipse>& ellipses_array)
{
  if (obs.param_list.empty())
    return;

  obs.obs_predict();

  for (int i = 0; i < obs.pred_list.size(); i++)
  {
    Ellipse ellipse;
    ellipse.cx = obs.pred_list[i].x;
    ellipse.cy = obs.pred_list[i].y;
    ellipse.semimajor = obs.pred_list[i].a;
    ellipse.semiminor = obs.pred_list[i].b;
    ellipse.theta = obs.pred_list[i].theta;
    ellipses_array.push_back(ellipse);

    obs_pub.data.push_back(obs.pred_list[i].x);
    obs_pub.data.push_back(obs.pred_list[i].y);
    obs_pub.data.push_back(obs.pred_list[i].a);
    obs_pub.data.push_back(obs.pred_list[i].b);
    obs_pub.data.push_back(obs.pred_list[i].theta);
  }
}

// 订阅/local_map_pub/for_obs_track话题的回调函数
void obscb(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  
  if (msg->data.empty())
    return;

  // 每个障碍物的数据格式：
  // msg->data[7*i + 0]  // x      - 障碍物中心x坐标
  // msg->data[7*i + 1]  // y      - 障碍物中心y坐标
  // msg->data[7*i + 2]  // a      - 椭圆长轴长度
  // msg->data[7*i + 3]  // b      - 椭圆短轴长度
  // msg->data[7*i + 4]  // theta  - 椭圆旋转角度
  // msg->data[7*i + 5]  // flag   - 障碍物ID标识
  // msg->data[7*i + 6]  // mea_cov- 测量协方差
  int num = msg->data.size() / 7;

  std_msgs::Float32MultiArray obs_pub;//定义要发布的预测障碍物数据

  vector<Ellipse> ellipses_array;//定义一个椭圆数组

  for (int i = 0; i < num; i++)
  {
    int flag = msg->data[7 * i + 5];

    obs_param _obs_tmp;
    _obs_tmp.x = msg->data[7 * i];
    _obs_tmp.y = msg->data[7 * i + 1];
    _obs_tmp.a = msg->data[7 * i + 2];
    _obs_tmp.b = msg->data[7 * i + 3];
    _obs_tmp.theta = msg->data[7 * i + 4];
    _obs_tmp.mea_cov = msg->data[7 * i + 6];

    _obs_kf[flag].param_list.push_back(_obs_tmp);

    curve_fitting(_obs_kf[flag], obs_pub, ellipses_array);
  }

  _obs_pub.publish(obs_pub);
  visEllipse(ellipses_array);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obs_kalman");
  ros::NodeHandle n;
  _obs_sub = n.subscribe("/lidar_node/for_obs_track", 1, obscb);
  _obs_pub = n.advertise<std_msgs::Float32MultiArray>("/obs_predict_pub", 1);
  _obs_vis_pub = n.advertise<visualization_msgs::MarkerArray>("/obs_predict_vis_pub", 1);
  ros::spin();
  return 0;
}
