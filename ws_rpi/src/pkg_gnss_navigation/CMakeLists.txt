cmake_minimum_required(VERSION 3.8)
project(pkg_gnss_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(action_ifaces REQUIRED)
find_package(msgs_ifaces REQUIRED)

include_directories(/usr/include/jsoncpp)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#------------------------------------------------------------------------------------#


add_executable(node_gnss_mission_monitor src/node_gnss_mission_monitor.cpp)
ament_target_dependencies(node_gnss_mission_monitor rclcpp rclcpp_action rclcpp_components std_msgs action_ifaces msgs_ifaces)
target_include_directories(node_gnss_mission_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(node_gnss_mission_monitor PUBLIC c_std_99 cxx_std_17)

install(TARGETS node_gnss_mission_monitor
  DESTINATION lib/${PROJECT_NAME})

#------------------------------------------------------------------------------------#

add_executable(node_gnss_spresense src/node_gnss_spresense.cpp)
ament_target_dependencies(node_gnss_spresense rclcpp std_msgs jsoncpp msgs_ifaces)

target_link_libraries(node_gnss_spresense jsoncpp)

target_include_directories(node_gnss_spresense PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS node_gnss_spresense
  DESTINATION lib/${PROJECT_NAME})

#------------------------------------------------------------------------------------#


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
