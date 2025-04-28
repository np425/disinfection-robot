cmake_minimum_required(VERSION 3.8)
project(uv_waypoint_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(uv_waypoint_executor SHARED src/uv_waypoint_executor.cpp)

target_compile_features(uv_waypoint_executor PUBLIC cxx_std_17)
target_include_directories(uv_waypoint_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(uv_waypoint_executor rclcpp lifecycle_msgs geometry_msgs pluginlib nav2_core)
pluginlib_export_plugin_description_file(nav2_waypoint_follower plugin.xml)

install(TARGETS uv_waypoint_executor
  EXPORT export_uv_waypoint_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

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

ament_export_targets(export_uv_waypoint_executor HAS_LIBRARY_TARGET)
ament_export_dependencies(
  nav2_core
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
