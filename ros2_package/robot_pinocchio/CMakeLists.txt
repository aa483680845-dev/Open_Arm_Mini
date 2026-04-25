cmake_minimum_required(VERSION 3.8)
project(robot_pinocchio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(COAL_DISABLE_HPP_FCL_WARNINGS ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(pinocchio_inverse SHARED src/pinocchio_inverse.cpp)
target_link_libraries(pinocchio_inverse pinocchio::pinocchio)
target_include_directories(pinocchio_inverse PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(pinocchio_main src/pinocchio_main.cpp)
ament_target_dependencies(pinocchio_main
  rclcpp
  ament_index_cpp
  std_msgs
  sensor_msgs
)
target_link_libraries(pinocchio_main pinocchio_inverse pinocchio::pinocchio)

install(DIRECTORY include DESTINATION include)
install(TARGETS pinocchio_inverse
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(TARGETS pinocchio_main DESTINATION lib/${PROJECT_NAME})

ament_package()
