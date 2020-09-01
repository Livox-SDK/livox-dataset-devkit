cmake_minimum_required(VERSION 3.0.2)
project(pcd_single_read)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
)
catkin_package(
 LIBRARIES pcd_single_read
 cv_bridge image_transport roscpp sensor_msgs std_msgs pcl_ros
 DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/pcd_single_read_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
