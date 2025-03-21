cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0148 OLD)
project(diffdrive_arduino)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

add_library(
  diffdrive_arduino 
  SHARED 
  src/diffdrive_arduino.cpp 
  src/wheel.cpp 
  src/arduino_comms.cpp
)

target_compile_features(diffdrive_arduino PUBLIC cxx_std_17)
target_include_directories(diffdrive_arduino PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/diffdrive_arduino>
)

ament_target_dependencies(
  diffdrive_arduino PUBLIC
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

target_link_libraries(diffdrive_arduino PUBLIC serial)
target_compile_definitions(${PROJECT_NAME} PRIVATE "DIFFDRIVE_ARDUINO_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)


install(
  DIRECTORY controllers launch/
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS diffdrive_arduino
  EXPORT export_diffdrive_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_diffdrive_arduino HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

ament_package()


