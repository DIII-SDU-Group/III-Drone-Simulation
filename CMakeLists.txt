cmake_minimum_required(VERSION 3.8)
project(iii_drone_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(CTest)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(iii_drone_core REQUIRED)
find_package(iii_drone_configuration REQUIRED)
find_package(iii_drone_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(depth_cam_to_mmwave src/depth_cam_to_mmwave.cpp)
ament_target_dependencies(depth_cam_to_mmwave
  rclcpp
  sensor_msgs
  iii_drone_core
  iii_drone_configuration
  iii_drone_interfaces
  pcl_ros
  pcl_conversions
)
install(TARGETS depth_cam_to_mmwave DESTINATION lib/${PROJECT_NAME})

############
# Install ##
############
# Export information to downstream packages
if(ROS_DISTRO IN_LIST ROS_DISTROS)
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs OpenCV)
else()
  ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime Eigen3 px4_msgs geometry_msgs sensor_msgs)
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(iii_drone_simulation_launch_smoke_test
    test/test_launch_smoke.py
  )
endif()

ament_package()
