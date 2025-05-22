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

# Verificar si existe el directorio de libsockcanpp
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/libsockcanpp-1.5.0")
    message(STATUS "Building sockcanpp from source")

    # Desactivar -Werror para libsockcanpp
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=ignored-qualifiers")

    # Añadir el subdirectorio directamente
    add_subdirectory(lib/libsockcanpp-1.5.0 ${CMAKE_CURRENT_BINARY_DIR}/libsockcanpp-1.5.0)
    
    # Definir variables para incluir y enlazar
    set(SOCKCANPP_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/lib/libsockcanpp-1.5.0/include")
    set(SOCKCANPP_LIBRARIES sockcanpp)
else()
    message(FATAL_ERROR "libsockcanpp directory not found at ${CMAKE_CURRENT_SOURCE_DIR}/lib/libsockcanpp-1.5.0")
endif()

# Compile options
add_compile_options(-DCADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER_BUILDING_DLL)

# Include directories
include_directories(include)

# Create the hardware plugin library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/curtis_motor_driver.cpp
  src/can_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${SOCKCANPP_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${SOCKCANPP_LIBRARIES}
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

# Install
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
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