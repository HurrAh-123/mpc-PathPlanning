#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include "lidar_pkg/ellipse.hpp"
#include "lidar_pkg/DBSCAN.hpp"
#include "lidar_pkg/KM.hpp"

using namespace grid_map;
using namespace std;

class LidarNode_N
{
private:
    ros::NodeHandle nh;

    const int robot_n = 5;  // 机器人数量

    float localmap_x_size;  
    float localmap_y_size;
    float resolution;       
    int block_size;        
    float obs_height;      
    float step_height;     
    float DBSCAN_R;       
    int DBSCAN_N;

    //tf变换相关
    tf::TransformListener listener;     //初始化tf监听器
    std::vector<tf::StampedTransform> base_link_transforms;//底盘
    std::vector<tf::StampedTransform> lidar_link_transforms;//雷达

    std::vector<Eigen::Vector2d> robot_positions;

    //话题的订阅和发布
    std::vector<ros::Subscriber> velodyne_subs;  // 订阅多个3维点云
    std::vector<ros::Publisher> local_pcd_pubs;//发布多个点云,转换成世界坐标系下的点云
    std::vector<ros::Publisher> for_obs_track_pubs;//发布多个障碍物跟踪信息

    //地图数据
    std::vector<GridMap> maps;
    std::vector<Eigen::MatrixXf> lidar_pcd_matrices;
    std::vector<Eigen::MatrixXf> elevation_matrices;
    std::vector<Eigen::MatrixXf> gradient_matrices;

    //点云数据
    std::vector<pcl::PointCloud<pcl::PointXYZ>> velodyne_clouds; // 存储多个点云数据（原始）
    std::vector<pcl::PointCloud<pcl::PointXYZ>> velodyne_clouds_filter;// 存储多个点云数据（经过滤波）
    std::vector<pcl::PointCloud<pcl::PointXYZ>> velodyne_clouds_global;// 存储多个点云数据（转换到世界坐标系）

    // 点云滤波器(用同一个就可以了)
    pcl::PassThrough<pcl::PointXYZ> pass_x; // X方向的直通滤波器，用于裁剪X方向上的点云
    pcl::PassThrough<pcl::PointXYZ> pass_y; // Y方向的直通滤波器，用于裁剪Y方向上的点云
    pcl::VoxelGrid<pcl::PointXYZ> sor; // 体素滤波器，用于降采样点云，减少数据量


public:
    LidarNode_N()
    {
      
      robot_positions.resize(robot_n);
      base_link_transforms.resize(robot_n);
      lidar_link_transforms.resize(robot_n);

      velodyne_clouds.resize(robot_n);
      velodyne_clouds_filter.resize(robot_n);
      velodyne_clouds_global.resize(robot_n);

      maps.resize(robot_n);
      lidar_pcd_matrices.resize(robot_n);
      elevation_matrices.resize(robot_n);
      gradient_matrices.resize(robot_n);

      //初始化sub和pub
      velodyne_subs.resize(robot_n);//初始化容器
      local_pcd_pubs.resize(robot_n);
      for_obs_track_pubs.resize(robot_n);


     
      //从参数服务器获取数据
      nh.param<float>("localmap_x_size", localmap_x_size, 10);//x方向10m
      nh.param<float>("localmap_y_size", localmap_y_size, 10);//y方向10m
      nh.param<float>("resolution", resolution, 0.1);//地图0.1m一格
      nh.param<int>("block_size", block_size, localmap_x_size /resolution * 0.2);//20

      nh.param<float>("obs_height", obs_height, 0.4);
      nh.param<float>("step_height", step_height, 0.5);

      nh.param<float>("DBSCAN_R", DBSCAN_R, 5.0);
      nh.param<int>("DBSCAN_N", DBSCAN_N, 5);

      
      for (int i = 0; i < robot_n; ++i)
      {
        velodyne_subs.push_back(nh.subscribe<sensor_msgs::PointCloud2>(
            "/robot" + std::to_string(i) + "/velodyne_points", 1, 
            [this,i](sensor_msgs::PointCloud2::ConstPtr msg) 
            { pcl::fromROSMsg(*msg, velodyne_clouds[i]); updateTF(i); }));

        local_pcd_pubs[i] = nh.advertise<sensor_msgs::PointCloud2>(
            "/robot" + std::to_string(i) + "/local_pcd", 1);
        for_obs_track_pubs[i] = nh.advertise<std_msgs::Float32MultiArray>(
            "/robot" + std::to_string(i) + "/for_obs_track", 1);
      }

      //初始化地图
      int map_index_len = localmap_x_size /resolution;//100,x方向格子数

      for (int i = 0; i < robot_n; ++i)
      {
        //local_lidar是高度地图数据
        //elevation是插值+膨胀后的local_lidar
        //gradient_map是梯度地图
        maps.push_back(GridMap({"local_lidar","elevation", "gradient_map"}));
        maps[i].setFrameId("world");//设置栅格地图的坐标系为"world"
        maps[i].setGeometry(Length(localmap_x_size, localmap_y_size), resolution);//长宽、格子大小
      
        //初始化存地图数据的矩阵
        lidar_pcd_matrices.push_back(Eigen::MatrixXf(map_index_len, map_index_len));
        elevation_matrices.push_back(Eigen::MatrixXf(map_index_len, map_index_len));
        gradient_matrices.push_back(Eigen::MatrixXf(map_index_len, map_index_len));
      }

      //点云滤波器参数设置
      pass_x.setFilterFieldName("x");
      pass_x.setFilterLimits(-localmap_x_size / 2, localmap_x_size / 2);
      pass_y.setFilterFieldName("y");
      pass_y.setFilterLimits(-localmap_y_size / 2, localmap_y_size / 2);
      sor.setLeafSize(0.05f, 0.05f, 0.05f);

    }

    //成员函数
    void updateTF(int robot_id);//更新tf
    void pcd_transform(int robot_id);//点云滤波+转换到世界坐标系
    void lidar2gridmap(Eigen::MatrixXf &lidar_data_matrix , int robot_id);//雷达信息转换成栅格地图
    
    Eigen::MatrixXf map_interpolation(const Eigen::MatrixXf &map_data);//对栅格地图进行插值处理
    void map_inflate_block(Eigen::MatrixXf &dst, const Eigen::MatrixXf &src, int startRow, int startCol, int radius);//对栅格地图进行膨胀处理
    Eigen::MatrixXf map_inflate(const Eigen::MatrixXf &map_data);//对栅格地图进行膨胀处理
    
    Eigen::MatrixXf gradient_map_processing(Eigen::MatrixXf &map_data, vector<DBSCAN::Point> &dataset);//用来检测和识别障碍物
    
    void loop_process(int robot_id);//主处理函数
    
    //成员变量
    KMAlgorithm KM;
};


// 更新机器人和激光雷达坐标
void LidarNode_N::updateTF(int robot_id)
{
  while (true)
  {
    try
    {
      listener.waitForTransform("world", "robot" + std::to_string(robot_id) + "/base_link", ros::Time(0), ros::Duration(0.1));
      listener.lookupTransform("world", "robot" + std::to_string(robot_id) + "/base_link", ros::Time(0), base_link_transforms[robot_id]);
      listener.waitForTransform("world", "robot" + std::to_string(robot_id) + "/lidar_link", ros::Time(0), ros::Duration(0.1));
      listener.lookupTransform("world", "robot" + std::to_string(robot_id) + "/lidar_link", ros::Time(0), lidar_link_transforms[robot_id]);

      // 更新机器人位置，雷达的要在三维点云转换成世界坐标系那个函数里面用
      robot_positions[robot_id] = Eigen::Vector2d(
        base_link_transforms[robot_id].getOrigin().x(), 
        base_link_transforms[robot_id].getOrigin().y());

      break;
    }
    catch (tf::TransformException &ex)
    {
      // ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }
}

void LidarNode_N::pcd_transform(int robot_id)
{
  // 对X方向进行直通滤波，结果存回velodyne_cloud，makeShared()创建一个点云的共享指针
  pass_x.setInputCloud(velodyne_clouds[robot_id].makeShared());
  pass_x.filter(velodyne_clouds[robot_id]);

  pass_y.setInputCloud(velodyne_clouds[robot_id].makeShared());
  pass_y.filter(velodyne_clouds[robot_id]);

  
  velodyne_clouds_filter[robot_id].clear();// 清空用于存储体素滤波结果的点云
  sor.setInputCloud(velodyne_clouds[robot_id].makeShared());// 将经过XY方向滤波的点云设置为体素滤波器的输入
  sor.filter(velodyne_clouds_filter[robot_id]);//进行体素滤波，结果存储在velodyne_cloud_filter中

  Eigen::Affine3d affine_transform;
  tf::transformTFToEigen(lidar_link_transforms[robot_id], affine_transform);
  pcl::transformPointCloud(velodyne_clouds_filter[robot_id], velodyne_clouds_global[robot_id], affine_transform);
}

//雷达信息转换成栅格地图，传入的是一个空矩阵，返回的是一个填充好的矩阵
void LidarNode_N::lidar2gridmap(Eigen::MatrixXf &lidar_data_matrix , int robot_id)
{
  int col = lidar_data_matrix.cols();//100
  int row = lidar_data_matrix.rows();//100
  for (const auto &pt : velodyne_clouds_global[robot_id])
  {
    int j = (pt.x - robot_positions[robot_id].x()) / resolution + col * 0.5;
    j = min(max(j, 0), row - 1);//确保计算出的索引不会超出地图边界
    int k = (pt.y - robot_positions[robot_id].y()) / resolution + row * 0.5;
    k = min(max(k, 0), col - 1);

    if (std::isnan(lidar_data_matrix(row - 1 - j, col - 1 - k)))
      lidar_data_matrix(row - 1 - j, col - 1 - k) = pt.z;
    if (lidar_data_matrix(row - 1 - j, col - 1 - k) < pt.z)
      lidar_data_matrix(row - 1 - j, col - 1 - k) = pt.z;
  }
}

//对栅格地图进行插值处理
Eigen::MatrixXf LidarNode_N::map_interpolation(const Eigen::MatrixXf &map_data)
{
  int col = map_data.cols(), row = map_data.rows();
  Eigen::MatrixXf map_interpolation(map_data);
  for (int i = 1; i < row - 1; i++)
  {
    for (int j = 1; j < col - 1; j++)
    {
      if (isnan(map_data(i, j)))
      {
        int count = 0;
        float height = 0;
        for (int k = 0; k <= 2; k++)
        {
          for (int q = 0; q <= 2; q++)
          {
            if (!isnan(map_data(i - 1 + k, j - 1 + q)))
            {
              count++;
              height += map_data(i - 1 + k, j - 1 + q);
            }
          }
        }
        map_interpolation(i, j) = (count > 0) ? height / count : NAN;
      }
    }
  }
  return map_interpolation;
}

// 对栅格地图进行膨胀处理
void LidarNode_N::map_inflate_block(Eigen::MatrixXf &dst, const Eigen::MatrixXf &src, int startRow, int startCol, int radius)
{
  for (int k = 0; k <= 2 * radius; k++)
  {
    for (int q = 0; q <= 2 * radius; q++)
    {
      if (isnan(src(startRow - radius + k, startCol - radius + q)))
      {
        dst(startRow - radius + k, startCol - radius + q) = src(startRow, startCol);
      }
    }
  }
}

// 对栅格地图进行膨胀处理
Eigen::MatrixXf LidarNode_N::map_inflate(const Eigen::MatrixXf &map_data)
{
  int col = map_data.cols(), row = map_data.rows();
  Eigen::MatrixXf map_inflated(map_data);
  for (int i = 3; i < row - 3; i++)
  {
    for (int j = 3; j < col - 3; j++)
    {
      if (isnan(map_data(i, j)))
        continue;

      double dis = sqrt((i - col / 2) * (i - col / 2) + (j - col / 2) * (j - col / 2));
      int radius;
      if (dis < col / 3)
        radius = 1;
      else if (dis < col * 0.45)
        radius = 2;
      else
        radius = 3;
      map_inflate_block(map_inflated, map_data, i, j, radius);
    }
  }
  return map_inflated;
}

//用来检测和识别障碍物
// 输入参数：

// 1. Eigen::MatrixXf &map_data ：
//    - 高度地图数据（插值和膨胀处理后的）
//    - 100×100的矩阵
//    - 每个元素表示对应位置的高度值

// 2. vector<DBSCAN::Point> &dataset ：
//    - 用于存储检测到的障碍物点
//    - 通过引用传递，函数会往里面添加障碍物点
//    - 每个Point包含(x,y)坐标信息

// 输出：

// 1. 返回值 Eigen::MatrixXf gradient_map ：
//    - 梯度地图，大小与输入地图相同(100×100)
//    - 每个点的值 = Sobel梯度值 + threshold
//    - 用于表示地形的变化程度

// 2. 通过引用修改的 dataset ：
//    - 包含所有被识别为障碍物的点
//    - 这些点满足以下任一条件：
//      - 梯度值大于obs_height + threshold（陡峭障碍物）
//      - 与周围最低点的高度差大于step_height（台阶障碍物）
Eigen::MatrixXf LidarNode_N::gradient_map_processing(Eigen::MatrixXf &map_data, vector<DBSCAN::Point> &dataset)
{
  const float threshold = -1.25;
  int col = map_data.cols(), row = map_data.rows();
  Eigen::MatrixXf gradient_map(row, col);
  gradient_map.setOnes();
  gradient_map *= threshold;
  DBSCAN::Point obs;

  for (int i = 1; i < row - 1; i++)
  {
    for (int j = 1; j < col - 1; j++)
    {
      bool has_nan_value = false;
      //检查当前点3x3邻域内是否存在无效值
      for (int p = -1; p <= 1; p++)
      {
        for (int q = -1; q <= 1; q++)
        {
          //// 如果发现无效值，将该点的梯度值设为threshold，并标记has_nan_value
          if (isnan(map_data(i + p, j + q)))
          {
            gradient_map(i, j) = threshold;
            has_nan_value = true;
          }
        }
      }
      // 如果邻域内没有无效值，进行Sobel算子计算
      if (!has_nan_value)
      {
        float sobel_x = map_data(i + 1, j + 1) - map_data(i - 1, j + 1) + 2*map_data(i + 1, j) - 2*map_data(i - 1, j) +
                        map_data(i + 1, j - 1) - map_data(i - 1, j - 1);
        float sobel_y = map_data(i - 1, j + 1) - map_data(i - 1, j - 1) + 2*map_data(i, j + 1) - 2*map_data(i, j - 1) +
                        map_data(i + 1, j + 1) - map_data(i + 1, j - 1);
        gradient_map(i, j) = sqrt(sobel_x * sobel_x + sobel_y * sobel_y) + threshold;
        if (gradient_map(i, j) > obs_height + threshold)
        {
          obs.x = i;
          obs.y = j;
          dataset.push_back(obs);//超过阈值的点判断为陡峭障碍物，加入dataset
        }
        else
        {
          // - 将地图分成若干个block（每个block是20×20的方格）
          // - 对于每个点，找到它所在block中的最低点
          // - 如果当前点与block中最低点的高度差超过step_height（默认0.5米），就认为这是一个台阶障碍物
          Eigen::MatrixXf::Index minRow, minCol;
          Eigen::MatrixXf block = map_data.block(i / block_size, j / block_size, block_size, block_size);
          float min = block.minCoeff(&minRow, &minCol);
          if (map_data(i, j) - min > step_height)
          {
            obs.x = i;
            obs.y = j;
            dataset.push_back(obs);//台阶障碍物和陡峭障碍物都被放进了dataset中
          }
        }
      }
    }
  }

  return gradient_map;
}


void LidarNode_N::loop_process(int robot_id)
{
  pcd_transform(robot_id);

  maps[robot_id].setPosition(robot_positions[robot_id]);
  maps[robot_id].clear("local_lidar");
  lidar_pcd_matrices[robot_id] = maps[robot_id].get("local_lidar");
  lidar2gridmap(lidar_pcd_matrices[robot_id], robot_id);

  maps[robot_id].add("local_lidar", lidar_pcd_matrices[robot_id]);

  elevation_matrices[robot_id] = map_interpolation(lidar_pcd_matrices[robot_id]);
  elevation_matrices[robot_id] = map_inflate(elevation_matrices[robot_id]);
  maps[robot_id].add("elevation", elevation_matrices[robot_id]);

   // obs map
  vector<DBSCAN::Point> non_clustered_obs;
  gradient_matrices[robot_id] = gradient_map_processing(elevation_matrices[robot_id], non_clustered_obs);

  // DBSCAN
  DBSCAN DS(DBSCAN_R, DBSCAN_N, non_clustered_obs);
  vector<Obstacle> clustered_obs(DS.cluster_num);
  for (const auto &obs : non_clustered_obs)
    {
      if (obs.obsID > 0)
      {
        gradient_matrices[robot_id](obs.x, obs.y) = -0.3;
        clustered_obs[obs.obsID - 1].emplace_back(obs.x, obs.y);
      }
    }    

  // get MSE椭圆方差计算
  vector<Ellipse> ellipses_array = get_ellipse_array(clustered_obs, maps[robot_id]);
  KM.tracking(ellipses_array);
  ab_variance_calculation(ellipses_array);
        
  // 发布障碍物跟踪信息
  std_msgs::Float32MultiArray for_obs_track;
  for (const auto &ellipse : ellipses_array)
    {
      if (ellipse.label == 0)
          continue;

      for_obs_track.data.push_back(ellipse.cx);
      for_obs_track.data.push_back(ellipse.cy);
      for_obs_track.data.push_back(ellipse.semimajor);
      for_obs_track.data.push_back(ellipse.semiminor);
      for_obs_track.data.push_back(ellipse.theta);
      for_obs_track.data.push_back(ellipse.label);
      for_obs_track.data.push_back(ellipse.variance);
    }
  for_obs_track_pubs[robot_id].publish(for_obs_track);

  // 发布世界坐标系下的点云数据
  sensor_msgs::PointCloud2 local_velodyne_msg;
  pcl::toROSMsg(velodyne_clouds_global[robot_id], local_velodyne_msg);
  local_velodyne_msg.header.stamp = ros::Time::now();
  local_velodyne_msg.header.frame_id = "world";
  local_pcd_pubs[robot_id].publish(local_velodyne_msg);




}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");//设置语言为中文
    ros::init(argc,argv,"lidar_node");
    ros::Time::init();

    LidarNode_N lidar_node;
    
    ros::Rate rate(10);

    

  
    while(ros::ok())
    {
      lidar_node.loop_process(0); 
      lidar_node.loop_process(1);
      lidar_node.loop_process(2);
      lidar_node.loop_process(3);
      lidar_node.loop_process(4);
     
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
