cmake_minimum_required(VERSION 3.10)
cmake_policy(SET CMP0167 NEW)
project(my_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_dm_driver REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
include_directories(include)
add_library(${PROJECT_NAME}
   SHARED 
  src/mobile_base_hardware_interfaces.cpp)
ament_target_dependencies(${PROJECT_NAME}  
  rclcpp
  robot_dm_driver
  hardware_interface
  rclcpp_lifecycle
  controller_interface
  pluginlib
)

pluginlib_export_plugin_description_file(
  hardware_interface
  my_robot_hardware_interface.xml
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
install(TARGETS ${PROJECT_NAME}  DESTINATION  lib/${PROJECT_NAME})
install(DIRECTORY include DESTINATION include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()
