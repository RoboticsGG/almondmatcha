cmake_minimum_required(VERSION 3.8)
project(pkg_sensdata)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs)
find_package(msgs_ifaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(node_sensdata src/node_sensdata.cpp)
ament_target_dependencies(node_sensdata rclcpp geometry_msgs std_msgs msgs_ifaces)

target_include_directories(node_sensdata PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(node_sensdata PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS node_sensdata
  DESTINATION lib/${PROJECT_NAME})

add_executable(node_imudata src/node_imudata.cpp)
ament_target_dependencies(node_imudata rclcpp geometry_msgs std_msgs msgs_ifaces)

target_include_directories(node_imudata PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(node_imudata PUBLIC c_std_99 cxx_std_17)

install(TARGETS node_imudata
  DESTINATION lib/${PROJECT_NAME})

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
