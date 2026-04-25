cmake_minimum_required(VERSION 3.8)
project(robot_dm_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

add_library(dm_motor SHARED
src/dm_motor.cpp
)
target_include_directories(dm_motor PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include>
 ${LIBUSB_INCLUDE_DIRS}
)

target_link_libraries(dm_motor PUBLIC 
-Wl,--no-as-needed      # 不要忽略未定义的符号
${LIBUSB_LIBRARIES}     # 注意顺序
${CMAKE_CURRENT_SOURCE_DIR}/lib/libdm_device.so
Threads::Threads
)

install(TARGETS dm_motor
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  FILES lib/libdm_device.so
  DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
