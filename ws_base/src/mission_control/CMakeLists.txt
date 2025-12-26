cmake_minimum_required(VERSION 3.8)
project(mission_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(services_ifaces REQUIRED)
find_package(action_ifaces REQUIRED)
find_package(msgs_ifaces REQUIRED)

# Mission Control Command Node
add_executable(mission_command_node src/node_commands.cpp)
ament_target_dependencies(mission_command_node 
  rclcpp 
  rclcpp_action 
  rclcpp_components 
  std_msgs 
  services_ifaces 
  action_ifaces
)

target_include_directories(mission_command_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(mission_command_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS mission_command_node
  DESTINATION lib/${PROJECT_NAME}
)

# Base Station Monitoring Display Node
add_executable(base_monitoring_node src/node_base_monitoring.cpp)
ament_target_dependencies(base_monitoring_node 
  rclcpp 
  msgs_ifaces
)

target_include_directories(base_monitoring_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(base_monitoring_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS base_monitoring_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration and launch files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
