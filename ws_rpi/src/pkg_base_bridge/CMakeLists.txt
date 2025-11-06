cmake_minimum_required(VERSION 3.8)
project(pkg_base_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(msgs_ifaces REQUIRED)

# Base Bridge Node executable
add_executable(node_base_bridge src/node_base_bridge.cpp)
ament_target_dependencies(node_base_bridge
  rclcpp
  std_msgs
  msgs_ifaces
)

# Install executables
install(TARGETS
  node_base_bridge
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
