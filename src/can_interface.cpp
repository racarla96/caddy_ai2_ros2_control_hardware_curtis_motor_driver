#include "caddy_ai2_ros2_control_hardware_curtis_motor_driver/can_interface.hpp"

#include <cstring>
#include <iostream>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cmath>

namespace caddy_ai2_ros2_control_hardware_curtis_motor_driver
{

#define CURTIS_SPEED_DIVISOR 10.0
#define KMH_TO_MPS 0.277778

CanInterface::CanInterface(const std::string & interface_name, rclcpp::Logger logger)
: interface_name_(interface_name), 
  logger_(logger), 
  is_open_(false),
  motor_direction_(-1),  // Default direction
  current_rms_(0.0),
  battery_current_(0.0),
  keyswitch_voltage_(0.0),
  bdi_percentage_(0.0),
  motor_rpm_(0),
  speed_(0.0),
  controller_temp_(0.0),
  motor_temp_(0.0),
  interlock_(false),
  on_fault_(false),
  fault_code_(0),
  mode_auto_(false),
  mode_manual_(false)
{
}

CanInterface::~CanInterface()
{
  close();
}

bool CanInterface::interfaceExists(const std::string & ifaceName) const
{
  unsigned int ifIndex = if_nametoindex(ifaceName.c_str());
  if (ifIndex == 0) {
    RCLCPP_ERROR(
      logger_, "Error al verificar interfaz %s: %s", 
      ifaceName.c_str(), strerror(errno));
  }
  return (ifIndex != 0);
}

bool CanInterface::configureInterface(int bitrate)
{
  if (!interfaceExists(interface_name_)) {
    RCLCPP_ERROR(
      logger_, "La interfaz %s no existe", 
      interface_name_.c_str());
    return false;
  }

  std::string cmd = "ip link set " + interface_name_ + " down 2>&1";
  int result = system(cmd.c_str());
  if (result != 0) {
    RCLCPP_WARN(
      logger_, "Advertencia al desactivar la interfaz: %s", 
      interface_name_.c_str());
  }

  cmd = "ip link set " + interface_name_ + " up type can bitrate " + 
        std::to_string(bitrate) + " 2>&1";
  result = system(cmd.c_str());
  if (result != 0) {
    RCLCPP_ERROR(
      logger_, "Error al configurar la interfaz %s con bitrate %d", 
      interface_name_.c_str(), bitrate);
    return false;
  }

  RCLCPP_INFO(
    logger_, "Interfaz CAN %s configurada correctamente con bitrate %d", 
    interface_name_.c_str(), bitrate);
  return true;
}

bool CanInterface::init()
{
  if (is_open_) {
    return true;
  }
  
  // Configurar la interfaz CAN
  if (!configureInterface()) {
    return false;
  }
  
  try {
    // Crear el driver CAN usando libsockcanpp
    can_driver_ = std::make_unique<sockcanpp::CanDriver>(interface_name_, CAN_RAW);
    is_open_ = true;
    RCLCPP_INFO(logger_, "Interfaz CAN %s inicializada correctamente", interface_name_.c_str());
    return true;
  } catch (const sockcanpp::CanInitException & e) {
    RCLCPP_ERROR(logger_, "Error al inicializar la interfaz CAN %s: %s", 
                interface_name_.c_str(), e.what());
    return false;
  }
}

bool CanInterface::close()
{
  if (!is_open_) {
    return true;
  }
  
  can_driver_.reset();
  is_open_ = false;
  RCLCPP_INFO(logger_, "Interfaz CAN %s cerrada", interface_name_.c_str());
  return true;
}

bool CanInterface::sendThrottle(double throttle_value, bool reset)
{
  if (!is_open_ || !can_driver_) {
    RCLCPP_ERROR(logger_, "No se puede enviar: interfaz CAN no abierta");
    return false;
  }
  
  // Limitar el valor del throttle entre -100 y 100
  throttle_value = std::max(-100.0, std::min(100.0, throttle_value));
  
  // Convertir a valor entero para enviar por CAN
  int16_t throttle_int = static_cast<int16_t>(throttle_value * 10.0);
  
  // Preparar datos para enviar
  uint8_t data[8] = {0};
  
  // Throttle en los primeros dos bytes (little endian)
  data[0] = (throttle_int & 0xFF);
  data[1] = ((throttle_int >> 8) & 0xFF);
  
  // Si es un reset, usar el código de reset
  if (reset) {
    data[2] = (RESET_CODE & 0xFF);
    data[3] = ((RESET_CODE >> 8) & 0xFF);
  } else {
    data[2] = 0;
    data[3] = 0;
  }
  
  // Crear mensaje CAN
  sockcanpp::CanMessage message(PDO1_MOSI_CtoM, data, 4);
  
  try {
    // Enviar mensaje CAN
    can_driver_->sendMessage(message);
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Error al enviar mensaje CAN: %s", e.what());
    return false;
  }
}

bool CanInterface::readStatus(double & velocity, double & current, double & temperature, int & fault_code)
{
  if (!is_open_ || !can_driver_) {
    RCLCPP_ERROR(logger_, "No se puede leer: interfaz CAN no abierta");
    return false;
  }
  
  bool status_received = false;
  
  // Verificar si hay mensajes disponibles
  if (can_driver_->waitForMessages(std::chrono::milliseconds(10))) {
    // Leer todos los mensajes disponibles
    auto messages = can_driver_->readQueuedMessages();
    
    while (!messages.empty()) {
      auto msg = messages.front();
      messages.pop();
      
      // Procesar según el ID del mensaje
      if (msg.getId() == PDO1_MISO_MtoC) {
        processPDO1_MISO_MtoC(msg);
        status_received = true;
      } else if (msg.getId() == PDO2_MISO_MtoC) {
        processPDO2_MISO_MtoC(msg);
        status_received = true;
      } else if (msg.getId() == PDO1_MOSI_MtoS) {
        processPDO1_MOSI_MtoS(msg);
        status_received = true;
      } else if (msg.getId() == PDO2_MISO_StoC) {
        processPDO2_MISO_StoC(msg);
        status_received = true;
      } else if (msg.getId() == PDO1_MISO_StoM) {
        processPDO1_MISO_StoM(msg);
        status_received = true;
      }
    }
  }
  
  // Actualizar valores de salida
  velocity = speed_;
  current = current_rms_;
  temperature = controller_temp_;
  fault_code = fault_code_;
  
  return status_received;
}

void CanInterface::processPDO1_MISO_MtoC(const sockcanpp::CanMessage & msg)
{
  const auto & data = msg.getData();
  
  // Extract the current rms value
  int16_t current = (data[1] << 8) | data[0];
  
  // Extract the battery_current value
  int16_t battery_current = (data[3] << 8) | data[2];
  
  // Extract the keyswitch_voltage value
  int16_t keyswitch_voltage = (data[5] << 8) | data[4];
  
  // Extract the bdi_percentage value
  int16_t bdi_percentage = (data[7] << 8) | data[6];
  
  current_rms_ = static_cast<double>(current) / 10.0;
  battery_current_ = static_cast<double>(battery_current) / 10.0;
  keyswitch_voltage_ = static_cast<double>(keyswitch_voltage) / 100.0;
  bdi_percentage_ = static_cast<double>(bdi_percentage);
}

void CanInterface::processPDO2_MISO_MtoC(const sockcanpp::CanMessage & msg)
{
  const auto & data = msg.getData();
  
  // Extract the RPM value
  int16_t rpm = (data[1] << 8) | data[0];
  
  // Extract the speed value
  int16_t speed = (data[3] << 8) | data[2];
  
  // Extract the controller temp value
  int16_t control_temp = (data[5] << 8) | data[4];
  
  // Extract the motor temp value
  int16_t motor_temp = (data[7] << 8) | data[6];
  
  motor_rpm_ = motor_direction_ * rpm;
  speed_ = motor_direction_ * (static_cast<double>(speed) / CURTIS_SPEED_DIVISOR) * KMH_TO_MPS;
  controller_temp_ = static_cast<double>(control_temp) / 10.0;
  motor_temp_ = static_cast<double>(motor_temp) / 10.0;
}

void CanInterface::processPDO1_MOSI_MtoS(const sockcanpp::CanMessage & msg)
{
  const auto & data = msg.getData();
  
  interlock_ = data[0] & 0x01;
  on_fault_ = (data[0] & 0x02) >> 1;
  fault_code_ = data[1];
  mode_auto_ = (data[0] & 0x04) >> 2;
  mode_manual_ = (data[0] & 0x08) >> 3;
}

void CanInterface::processPDO2_MISO_StoC(const sockcanpp::CanMessage & msg)
{
  // Esta función procesa datos del controlador esclavo
  // No es necesaria para la interfaz básica del actuador
}

void CanInterface::processPDO1_MISO_StoM(const sockcanpp::CanMessage & msg)
{
  // Esta función procesa datos del controlador esclavo
  // No es necesaria para la interfaz básica del actuador
}

}  // namespace caddy_ai2_ros2_control_hardware_curtis_motor_driver