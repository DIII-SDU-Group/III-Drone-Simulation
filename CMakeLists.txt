cmake_minimum_required(VERSION 3.8)
project(iii_drone_simulation)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
  set(CMAKE_POLICY_DEFAULT_CMP0144 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(CTest)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(iii_drone_core REQUIRED)
find_package(iii_drone_configuration REQUIRED)
find_package(iii_drone_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED COMPONENTS io)
find_package(gz-math7 REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(gz-sim8 QUIET)
if(gz-sim8_FOUND)
  find_package(gz-msgs10 REQUIRED)
  find_package(gz-transport13 REQUIRED)
  set(III_DRONE_SIMULATION_GZ_SIM_TARGET gz-sim8::core)
  set(III_DRONE_SIMULATION_GZ_MSGS_TARGET gz-msgs10::core)
  set(III_DRONE_SIMULATION_GZ_TRANSPORT_TARGET gz-transport13::core)
else()
  find_package(gz-sim7 REQUIRED)
  find_package(gz-msgs9 REQUIRED)
  find_package(gz-transport12 REQUIRED)
  set(III_DRONE_SIMULATION_GZ_SIM_TARGET gz-sim7::core)
  set(III_DRONE_SIMULATION_GZ_MSGS_TARGET gz-msgs9::core)
  set(III_DRONE_SIMULATION_GZ_TRANSPORT_TARGET gz-transport12::core)
endif()

if(TARGET yaml-cpp::yaml-cpp)
  set(III_DRONE_SIMULATION_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(III_DRONE_SIMULATION_YAML_CPP_TARGET yaml-cpp)
else()
  message(FATAL_ERROR "yaml-cpp target not found")
endif()

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

add_executable(sim_charger_gripper_node src/sim_charger_gripper_node.cpp)
target_compile_features(sim_charger_gripper_node PRIVATE cxx_std_17)
ament_target_dependencies(sim_charger_gripper_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  iii_drone_interfaces
  px4_msgs
)
target_link_libraries(sim_charger_gripper_node
  ${III_DRONE_SIMULATION_GZ_MSGS_TARGET}
  ${III_DRONE_SIMULATION_GZ_TRANSPORT_TARGET}
)
install(TARGETS sim_charger_gripper_node DESTINATION lib/${PROJECT_NAME})

add_library(iii_drone_mmwave_conductor_sensor_plugin SHARED
  src/mmwave_conductor_sensor_plugin.cpp
)
target_compile_features(iii_drone_mmwave_conductor_sensor_plugin PRIVATE cxx_std_17)
target_include_directories(iii_drone_mmwave_conductor_sensor_plugin PRIVATE include)
target_link_libraries(iii_drone_mmwave_conductor_sensor_plugin
    ${III_DRONE_SIMULATION_GZ_SIM_TARGET}
    gz-plugin2::requested
    ${III_DRONE_SIMULATION_GZ_MSGS_TARGET}
    ${III_DRONE_SIMULATION_GZ_TRANSPORT_TARGET}
    gz-common5::requested
    gz-math7::core
    ${III_DRONE_SIMULATION_YAML_CPP_TARGET}
)
ament_target_dependencies(iii_drone_mmwave_conductor_sensor_plugin
  rclcpp
  sensor_msgs
  iii_drone_interfaces
)
install(TARGETS iii_drone_mmwave_conductor_sensor_plugin DESTINATION lib)

# The simulator-v2 radar model is header-only and compiled into the plugin
# translation unit; unoptimised it runs about ten times slower in Gazebo.
option(III_DRONE_SIMULATION_OPTIMIZE_RADAR_KERNELS
  "Optimize the mmWave sensor plugin radar kernels in Debug builds" ON)
if(III_DRONE_SIMULATION_OPTIMIZE_RADAR_KERNELS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set_property(SOURCE
    "${PROJECT_SOURCE_DIR}/src/mmwave_conductor_sensor_plugin.cpp"
    APPEND PROPERTY COMPILE_OPTIONS "$<$<CONFIG:Debug>:-O2>"
  )
endif()

# Optional CUDA backend for the simulator-v2 radar model. The CPU reference is
# always built; the backend is selected at runtime (processing.backend or
# III_AOP_BACKEND) and falls back to the CPU when no device is available.
option(III_DRONE_SIMULATION_CUDA "Build the optional CUDA simulator-v2 backend" OFF)
set(III_DRONE_SIMULATION_CUDA_ARCHITECTURES "75" CACHE STRING
  "CUDA architectures for the optional simulator-v2 backend")
if(III_DRONE_SIMULATION_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
    add_library(iii_drone_aop_radar_cuda STATIC src/aop_radar_cuda.cu)
    set_target_properties(iii_drone_aop_radar_cuda PROPERTIES
      CUDA_ARCHITECTURES "${III_DRONE_SIMULATION_CUDA_ARCHITECTURES}"
      POSITION_INDEPENDENT_CODE ON
      CUDA_STANDARD 17
      CUDA_STANDARD_REQUIRED ON)
    target_include_directories(iii_drone_aop_radar_cuda PRIVATE include)
    target_compile_definitions(iii_drone_aop_radar_cuda PRIVATE AOP_WITH_CUDA)
    target_compile_options(iii_drone_aop_radar_cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-fmad=false>)
    target_compile_definitions(iii_drone_mmwave_conductor_sensor_plugin PRIVATE AOP_WITH_CUDA)
    target_link_libraries(iii_drone_mmwave_conductor_sensor_plugin iii_drone_aop_radar_cuda CUDA::cudart)
    message(STATUS "iii_drone_simulation: CUDA simulator-v2 backend enabled")
  else()
    message(STATUS "iii_drone_simulation: no CUDA compiler; CPU simulator-v2 backend only")
  endif()
endif()

add_library(iii_drone_sim_charger_gripper_plugin SHARED
  src/sim_charger_gripper_plugin.cpp
)
target_compile_features(iii_drone_sim_charger_gripper_plugin PRIVATE cxx_std_17)
target_include_directories(iii_drone_sim_charger_gripper_plugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(iii_drone_sim_charger_gripper_plugin
  PRIVATE
    ${III_DRONE_SIMULATION_GZ_SIM_TARGET}
    gz-plugin2::requested
    ${III_DRONE_SIMULATION_GZ_MSGS_TARGET}
    ${III_DRONE_SIMULATION_GZ_TRANSPORT_TARGET}
    gz-common5::requested
    gz-math7::core
    ${III_DRONE_SIMULATION_YAML_CPP_TARGET}
)
install(TARGETS iii_drone_sim_charger_gripper_plugin DESTINATION lib)

############
# Install ##
############
# Export information to downstream packages
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  iii_drone_simulation/ground_truth_frame_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ground_truth_frame_broadcaster
)

install(PROGRAMS
  iii_drone_simulation/rate_limited_image_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rate_limited_image_relay
)

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
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(iii_drone_simulation_charger_gripper_jaws_test
    test/charger_gripper_jaws_test.cpp
  )
  if(TARGET iii_drone_simulation_charger_gripper_jaws_test)
    target_compile_features(iii_drone_simulation_charger_gripper_jaws_test PRIVATE cxx_std_17)
    target_include_directories(iii_drone_simulation_charger_gripper_jaws_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(iii_drone_simulation_charger_gripper_jaws_test gz-math7::core)
  endif()
  ament_add_gtest(iii_drone_simulation_mmwave_fov_geometry_test
    test/mmwave_fov_geometry_test.cpp
  )
  if(TARGET iii_drone_simulation_mmwave_fov_geometry_test)
    target_compile_features(iii_drone_simulation_mmwave_fov_geometry_test PRIVATE cxx_std_17)
    target_include_directories(iii_drone_simulation_mmwave_fov_geometry_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(iii_drone_simulation_launch_smoke_test
    test/test_launch_smoke.py
  )
endif()

ament_package()
