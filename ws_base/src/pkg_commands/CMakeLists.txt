cmake_minimum_required(VERSION 3.8)
project(pkg_commands)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs)
find_package(services_ifaces REQUIRED)
find_package(action_ifaces REQUIRED)
find_package(msgs_ifaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#-------------------node_commands-------------------
add_executable(node_commands src/node_commands.cpp)
ament_target_dependencies(node_commands rclcpp rclcpp_action rclcpp_components std_msgs services_ifaces action_ifaces)

target_include_directories(node_commands PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(node_commands PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS node_commands
  DESTINATION lib/${PROJECT_NAME})

  install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
#------------------node_commands-------------------
#-------------------node_monotoring----------------
add_executable(node_monitoring src/node_monitoring.cpp)
ament_target_dependencies(node_monitoring rclcpp std_msgs msgs_ifaces)

install(TARGETS
node_monitoring
  DESTINATION lib/${PROJECT_NAME}
)
#-------------------node_monotoring----------------

ament_package()
