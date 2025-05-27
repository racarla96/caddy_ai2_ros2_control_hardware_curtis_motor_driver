cmake_minimum_required(VERSION 3.8)
project(caddy_ai2_ros2_control_hardware_curtis_motor_driver)

# Establecer el estándar C++20 para todo el proyecto
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Compile options
add_compile_options(-DCADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER_BUILDING_DLL)

# Include directories
include_directories(hardware/include/caddy_ai2_ros2_control_hardware_curtis_motor_driver)

# Create the hardware plugin library
add_library(
  ${PROJECT_NAME}
  SHARED
  hardware/src/socket_can_interface.cpp
  hardware/src/curtis_motor_driver.cpp
  hardware/src/curtis_motor_hardware_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

target_link_libraries(
  ${PROJECT_NAME}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)


install(
  DIRECTORY hardware/include/
  DESTINATION include
)

install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)

# Export package dependencies
ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()