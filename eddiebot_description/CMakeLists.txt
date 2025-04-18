cmake_minimum_required(VERSION 3.8)
project(eddiebot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# find_package(ros2_control_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  eddiebot_description
  SHARED
  hardware/diffbot_system.cpp
)

target_compile_features(eddiebot_description PUBLIC cxx_std_17)
target_include_directories(eddiebot_description PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/eddiebot_description>
)
ament_target_dependencies(
  eddiebot_description PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(eddiebot_description PUBLIC serial)

pluginlib_export_plugin_description_file(hardware_interface eddiebot_description.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/eddiebot_description
)

install(
  DIRECTORY launch meshes urdf config
  DESTINATION share/eddiebot_description
)

install(TARGETS eddiebot_description
  EXPORT export_eddiebot_description
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_targets(export_eddiebot_description HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
