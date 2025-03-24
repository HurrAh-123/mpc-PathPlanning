cmake_minimum_required(VERSION 3.0.2)
project(my_robot)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  urdf
)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_robot
#  CATKIN_DEPENDS gazebo_ros roscpp rospy urdf
#  DEPENDS system_lib
)


include_directories(
  include/my_robot
  ${catkin_INCLUDE_DIRS}
)

add_executable(movetest src/movetest.cpp)

target_link_libraries(movetest
  ${catkin_LIBRARIES}
)
