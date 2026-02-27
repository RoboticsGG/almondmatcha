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
add_executable(mission_command_node src/mission_command_node.cpp)
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

# Mission Monitoring Node (PC) - Telemetry Relay Subscriber
add_executable(mission_monitoring_node_pc src/mission_monitoring_node_pc.cpp)
ament_target_dependencies(mission_monitoring_node_pc 
  rclcpp 
  msgs_ifaces
)

target_include_directories(mission_monitoring_node_pc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(mission_monitoring_node_pc PUBLIC c_std_99 cxx_std_17)

install(TARGETS mission_monitoring_node_pc
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
