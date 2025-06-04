cmake_minimum_required(VERSION 3.16)
project(caddy_ai2_ros2_control_hardware_curtis_motor_driver LANGUAGES CXX)

# Establecer el estándar C++20 para todo el proyecto
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#find_package(ros2_control_cmake REQUIRED)
#set_compiler_options()
# export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  caddy_ai2_ros2_control_hardware_curtis_motor_driver
  SHARED
  hardware/src/curtis_motor_hardware_interface.cpp
  hardware/src/curtis_motor_driver.cpp
  hardware/src/socket_can_interface.cpp
)
# target_include_directories(caddy_ai2_ros2_control_hardware_curtis_motor_driver PUBLIC
# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
# $<INSTALL_INTERFACE:include/caddy_ai2_ros2_control_hardware_curtis_motor_driver>
# )
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(caddy_ai2_ros2_control_hardware_curtis_motor_driver PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/caddy_ai2_ros2_control_hardware_curtis_motor_driver
)
install(
  # DIRECTORY description/launch description/ros2_control description/urdf
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/caddy_ai2_ros2_control_hardware_curtis_motor_driver
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/caddy_ai2_ros2_control_hardware_curtis_motor_driver
)
install(TARGETS caddy_ai2_ros2_control_hardware_curtis_motor_driver
  EXPORT caddy_ai2_ros2_control_hardware_curtis_motor_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# if(BUILD_TESTING)
#   find_package(ament_cmake_pytest REQUIRED)

#   ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
#   ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
#   ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
# endif()

## EXPORTS
ament_export_targets(caddy_ai2_ros2_control_hardware_curtis_motor_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
