cmake_minimum_required(VERSION 3.8)
project(robot_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)

add_library(motor_controller SHARED
  src/motor_controller.cpp
)

target_include_directories(motor_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(motor_controller
  rclcpp
  rclcpp_lifecycle
  pluginlib
  controller_interface
  hardware_interface
  realtime_tools
  example_interfaces
  std_msgs
)

target_link_libraries(motor_controller pinocchio::pinocchio)

pluginlib_export_plugin_description_file(controller_interface motor_controller_plugin.xml)

install(
  TARGETS motor_controller
  EXPORT export_motor_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(motor_controller)
ament_export_targets(export_motor_controller)

ament_package()
