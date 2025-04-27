cmake_minimum_required(VERSION 3.8)
project(uv_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(uv_msgs REQUIRED)   

add_library(uv_controller
  SHARED
  src/uv_controller.cpp
)

target_include_directories(uv_controller PRIVATE include)
ament_target_dependencies(uv_controller
  controller_interface
  rclcpp
  rclcpp_lifecycle
  uv_msgs    
)
pluginlib_export_plugin_description_file(controller_interface uv_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS uv_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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



ament_export_dependencies(
  controller_interface
  rclcpp
  rclcpp_lifecycle
  uv_msgs    
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  uv_controller
)
ament_package()
