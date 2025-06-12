cmake_minimum_required(VERSION 3.8)
project(pointcloud_concatenate)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/pointcloud_concatenate.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pcl_ros
  sensor_msgs
  tf2_ros
)

add_executable(pointcloud_concatenate_node
  src/pointcloud_concatenate_node.cpp
)
ament_target_dependencies(pointcloud_concatenate_node
  rclcpp
  pcl_ros
  sensor_msgs
  tf2_ros
  pcl_conversions
)
target_link_libraries(pointcloud_concatenate_node
  ${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  pointcloud_concatenate_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
