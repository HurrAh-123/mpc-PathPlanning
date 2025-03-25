cmake_minimum_required(VERSION 3.0.2)
project(obs_kalman)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs 
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(obs_kalman src/obs_kalman.cpp src/kalman.cpp)

target_link_libraries(obs_kalman
  ${catkin_LIBRARIES}
)


