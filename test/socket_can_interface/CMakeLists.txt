cmake_minimum_required(VERSION 3.8)
project(test_socket_can_interface)

# Configuración de C++
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Buscar libsocketcan
find_library(SOCKETCAN_LIBRARY socketcan REQUIRED)

# Incluir directorios
include_directories(include)

# Crear la biblioteca
add_library(socket_can_interface
  src/socket_can_interface.cpp
)

# Vincular libsocketcan a la biblioteca
target_link_libraries(socket_can_interface
  ${SOCKETCAN_LIBRARY}
)

# Crear el ejecutable
add_executable(test_socket_can_interface_reader
  src/main.cpp
)

# Vincular la biblioteca al ejecutable
target_link_libraries(test_socket_can_interface_reader
socket_can_interface
)