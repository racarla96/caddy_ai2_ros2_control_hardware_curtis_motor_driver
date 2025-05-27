#include "socket_can_interface.hpp"
#include "curtis_motor_driver.hpp"
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <chrono>
#include <thread>



// Variable global para manejar señales
volatile bool running = true;

// Manejador de señales para salir limpiamente
void signalHandler(int signum) {
  std::cout << "Señal recibida, saliendo..." << std::endl;
  running = false;
}

// Función para imprimir un mensaje CAN
void printCANFrame(const struct can_frame& frame) {
  std::cout << "ID: 0x" << std::hex << std::setw(3) << std::setfill('0') 
            << frame.can_id << " [" << std::dec << (int)frame.can_dlc << "] ";
  
  for (int i = 0; i < frame.can_dlc; i++) {
    std::cout << std::hex << std::setw(2) << std::setfill('0') 
              << (int)frame.data[i] << " ";
  }
  
  std::cout << std::dec << std::endl;
}

int main(int argc, char** argv) {
  // Tipo de comportamiento
  int tipo_bucle = 0; // 0: a todo trapo

  // Configurar manejador de señales
  signal(SIGINT, signalHandler);
  
  // Obtener nombre de interfaz de argumentos o usar valor por defecto
  std::string interface_name = "can0";
  if (argc > 1) {
    interface_name = argv[1];
  }
  
  std::cout << "Iniciando lector SocketCAN en interfaz " << interface_name << std::endl;
  
  // Crear e inicializar interfaz SocketCAN
  SocketCANInterface can_interface(interface_name);
  CurtisMotorDriver curtis_motor_driver(true);
  
  if (!can_interface.init()) {
    std::cerr << "Error al inicializar interfaz SocketCAN" << std::endl;
    return 1;
  }
  
  std::cout << "Esperando mensajes CAN..." << std::endl;
  
  if(tipo_bucle == 0) {
    // Bucle principal
    while (running) {
      std::vector<struct can_frame> frames;
      int num_frames = can_interface.read(frames, 0);

      if (num_frames > 0) {
        curtis_motor_driver.process_frames(frames);
      } else if (num_frames < 0) {
        std::cerr << "Error al leer mensajes CAN" << std::endl;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 1000 Hz
    }
  }
  
  std::cout << "Cerrando interfaz SocketCAN..." << std::endl;
  can_interface.close();
  
  return 0;
}