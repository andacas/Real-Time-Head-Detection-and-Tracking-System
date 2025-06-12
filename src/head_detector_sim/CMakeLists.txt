cmake_minimum_required(VERSION 3.18)
project(head_detector_sim)   # no code, but keep C++

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)              # launch description may reference it
find_package(ros_gz_sim REQUIRED)          # Gazebo/Ignition simulator
find_package(ros_gz_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rviz2 REQUIRED)


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)
ament_package()








