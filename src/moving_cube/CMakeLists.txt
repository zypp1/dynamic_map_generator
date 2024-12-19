cmake_minimum_required(VERSION 3.0.2)
project(moving_cube)

set(CMAKE_BUILD_TYPE "Release")
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
)

catkin_package()

add_executable(moving_cube src/moving_cube.cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  moving_cube
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

