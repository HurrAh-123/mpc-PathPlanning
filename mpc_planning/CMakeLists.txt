cmake_minimum_required(VERSION 3.0.2)
project(mpc_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  eigen_conversions
  tf2_ros
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpc_planning
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs eigen_conversions tf2_ros
  DEPENDS EIGEN3 
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/local/include/qpOASES
)

# 添加库文件
add_library(mpc_planning
  src/uav_model.cpp
  src/mpc_controller.cpp
  
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  qpOASES
)

# 添加可执行文件
add_executable(mpc_planning_node src/mpc_planning_node.cpp)


target_link_libraries(mpc_planning_node
  mpc_planning
  ${catkin_LIBRARIES}
) 

