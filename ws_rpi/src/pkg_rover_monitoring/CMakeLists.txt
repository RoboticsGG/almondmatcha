cmake_minimum_required(VERSION 3.8)
project(pkg_rover_monitoring)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(msgs_ifaces REQUIRED)

add_executable(rover_monitoring_node src/rover_monitoring_node.cpp)
ament_target_dependencies(rover_monitoring_node rclcpp std_msgs msgs_ifaces)

add_executable(mission_monitoring_node_rpi src/mission_monitoring_node_rpi.cpp)
ament_target_dependencies(mission_monitoring_node_rpi rclcpp std_msgs msgs_ifaces)

# Install executables
install(TARGETS
  rover_monitoring_node
  mission_monitoring_node_rpi
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
