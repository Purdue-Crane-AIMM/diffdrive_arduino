cmake_minimum_required(VERSION 3.5)
project(diffdrive_arduino)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(
  diffdrive_arduino 
  SHARED 
  src/diffdrive_arduino.cpp 
  src/wheel.cpp 
  src/arduino_comms.cpp
)

target_compile_features(diffdrive_arduino PUBLIC cxx_std_14)
target_include_directories(diffdrive_arduino PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/diffdrive_arduino>
)

ament_target_dependencies(
  diffdrive_arduino PUBLIC
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  pluginlib
)

target_link_libraries(diffdrive_arduino PUBLIC serial)
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFFDRIVE_ARDUINO_BUILDING_DLL")

install(DIRECTORY launch config include 
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS diffdrive_arduino
  EXPORT export_diffdrive_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_diffdrive_arduino HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  pluginlib
)
ament_package()



add_executable(diffdrive_arduino_node src/diffdrive_arduino.cpp)
target_link_libraries(diffdrive_arduino_node PUBLIC
  diffdrive_arduino  # link against your shared lib
)

ament_target_dependencies(diffdrive_arduino_node PUBLIC
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
  pluginlib
)

install(TARGETS
  diffdrive_arduino_node 
  DESTINATION lib/${PROJECT_NAME}
)