#ifndef CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER__CAN_INTERFACE_HPP_
#define CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER__CAN_INTERFACE_HPP_

#include <string>
#include <memory>
#include <chrono>
#include <CanDriver.hpp>  // libsockcanpp
#include <exceptions/CanException.hpp>
#include <exceptions/CanInitException.hpp>
#include <exceptions/InvalidSocketException.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"

namespace caddy_ai2_ros2_control_hardware_curtis_motor_driver
{

// PDO IDs para mensajes CAN (adaptados del código original)
static const uint32_t PDO1_MISO_MtoC = 0x1A6;  // Master to Computer
static const uint32_t PDO1_MISO_StoM = 0x1A7;  // Slave to Master
static const uint32_t PDO1_MOSI_MtoS = 0x227;  // Master to Slave
static const uint32_t PDO1_MOSI_CtoM = 0x226;  // Computer to Master
static const uint32_t PDO2_MISO_StoC = 0x2A7;  // Slave to Computer
static const uint32_t PDO2_MISO_MtoC = 0x2A6;  // Master to Computer
static const uint32_t RESET_CODE = 0x4850;     // Código de reset

class CanInterface
{
public:
  CanInterface(const std::string & interface_name, rclcpp::Logger logger);
  ~CanInterface();

  bool init();
  bool close();
  bool configureInterface(int bitrate = 100000);
  bool sendThrottle(double throttle_value, bool reset = false);
  bool readStatus(double & velocity, double & current, double & temperature, int & fault_code);
  
  // Getters para datos completos
  double getCurrentRMS() const { return current_rms_; }
  double getBatteryCurrent() const { return battery_current_; }
  double getKeyswitchVoltage() const { return keyswitch_voltage_; }
  double getBDIPercentage() const { return bdi_percentage_; }
  double getMotorRPM() const { return motor_rpm_; }
  double getSpeed() const { return speed_; }
  double getControllerTemp() const { return controller_temp_; }
  double getMotorTemp() const { return motor_temp_; }
  bool getInterlock() const { return interlock_; }
  bool getOnFault() const { return on_fault_; }
  int getFaultCode() const { return fault_code_; }
  bool getModeAuto() const { return mode_auto_; }
  bool getModeManual() const { return mode_manual_; }

private:
  std::string interface_name_;
  rclcpp::Logger logger_;
  std::unique_ptr<sockcanpp::CanDriver> can_driver_;
  bool is_open_;
  int motor_direction_;
  
  // Datos del motor
  double current_rms_;
  double battery_current_;
  double keyswitch_voltage_;
  double bdi_percentage_;
  double motor_rpm_;
  double speed_;
  double controller_temp_;
  double motor_temp_;
  bool interlock_;
  bool on_fault_;
  int fault_code_;
  bool mode_auto_;
  bool mode_manual_;
  
  // Métodos para procesar mensajes CAN
  void processPDO1_MISO_MtoC(const sockcanpp::CanMessage & msg);
  void processPDO2_MISO_MtoC(const sockcanpp::CanMessage & msg);
  void processPDO1_MOSI_MtoS(const sockcanpp::CanMessage & msg);
  void processPDO2_MISO_StoC(const sockcanpp::CanMessage & msg);
  void processPDO1_MISO_StoM(const sockcanpp::CanMessage & msg);
  
  // Método para verificar si la interfaz existe
  bool interfaceExists(const std::string & ifaceName) const;
};

}  // namespace caddy_ai2_ros2_control_hardware_curtis_motor_driver

#endif  // CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER__CAN_INTERFACE_HPP_