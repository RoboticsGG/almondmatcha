cmake_minimum_required(VERSION 3.8)
project(pkg_gnss_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(msgs_ifaces REQUIRED)

# Optional dependencies for mission monitor
find_package(rclcpp_action QUIET)
find_package(rclcpp_components QUIET)
find_package(action_ifaces QUIET)

include_directories(/usr/include/jsoncpp)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#------------------------------------------------------------------------------------#

# Build mission monitor only if action_ifaces is available
if(action_ifaces_FOUND AND rclcpp_action_FOUND AND rclcpp_components_FOUND)
  add_executable(gnss_mission_monitor_node src/gnss_mission_monitor_node.cpp)
  ament_target_dependencies(gnss_mission_monitor_node rclcpp rclcpp_action rclcpp_components std_msgs action_ifaces msgs_ifaces)
  target_include_directories(gnss_mission_monitor_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(gnss_mission_monitor_node PUBLIC c_std_99 cxx_std_17)

  install(TARGETS gnss_mission_monitor_node
    DESTINATION lib/${PROJECT_NAME})
  
  message(STATUS "Building gnss_mission_monitor_node")
else()
  message(STATUS "Skipping gnss_mission_monitor_node (action_ifaces not found)")
endif()

#------------------------------------------------------------------------------------#

add_executable(gnss_spresense_node src/gnss_spresense_node.cpp)
ament_target_dependencies(gnss_spresense_node rclcpp std_msgs jsoncpp msgs_ifaces)

target_link_libraries(gnss_spresense_node jsoncpp)

target_include_directories(gnss_spresense_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS gnss_spresense_node
  DESTINATION lib/${PROJECT_NAME})

#------------------------------------------------------------------------------------#

add_executable(gnss_ublox_node src/gnss_ublox_node.cpp)
ament_target_dependencies(gnss_ublox_node rclcpp std_msgs msgs_ifaces)

target_include_directories(gnss_ublox_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS gnss_ublox_node
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
