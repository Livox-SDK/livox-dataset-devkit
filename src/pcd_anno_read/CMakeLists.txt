cmake_minimum_required(VERSION 2.8.3)
project(pcd_anno_read)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
 LIBRARIES pcd_anno_read
 CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
 DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/pcd_anno_read_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
