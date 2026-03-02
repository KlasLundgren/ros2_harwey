cmake_minimum_required(VERSION 3.8)
project(navigation_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Generate master.dcf and master.bin from bus.yml + EDS files
generate_dcf(bot_bus)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Build the hardware interface shared library
add_library(dmke_can_hw_interface SHARED
  src/dmke_can_hw_interface.cpp
)
target_include_directories(dmke_can_hw_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dmke_can_hw_interface
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  pcl_conversions
  pcl_ros
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

install(DIRECTORY
  launch
  config
  description
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Register the plugin with pluginlib
# WHY this macro: it writes the plugin path into the ament index so
# controller_manager can find it at runtime by class name.
pluginlib_export_plugin_description_file(hardware_interface dmke_can_hw_plugin.xml)

# Install the library
install(TARGETS dmke_can_hw_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install config, launch, urdf files
install(DIRECTORY config launch description
  DESTINATION share/${PROJECT_NAME}
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

ament_package()
