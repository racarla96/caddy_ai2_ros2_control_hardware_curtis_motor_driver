#ifndef CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER__CURTIS_MOTOR_DRIVER_HPP_
#define CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER__CURTIS_MOTOR_DRIVER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "caddy_ai2_ros2_control_hardware_curtis_motor_driver/visibility_control.h"
#include "caddy_ai2_ros2_control_hardware_curtis_motor_driver/can_interface.hpp"

// Constantes
#define DEFAULT_CAN_PORT "can0"
#define WATCHDOG_TIMEOUT 0.2  // segundos
#define MOTOR_DIRECTION -1    // +1 REGULAR, -1 INVERTED
#define RESET_CODE 0x4850

namespace caddy_ai2_ros2_control_hardware_curtis_motor_driver
{

// Estructura para parámetros del motor
struct CurtisMotorParams {
  double wheel_diameter;
  double max_rpm;
  double max_rps;
  double gear_ratio;
  double max_v_mps;
  double max_v_auto;
  int motor_direction;
};

class CurtisMotorDriver : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CurtisMotorDriver);

  CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER_PUBLIC
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER_PUBLIC
  hardware_interface::return_type read() override;

  CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER_PUBLIC
  hardware_interface::return_type write() override;

private:
  // Parámetros
  std::string can_interface_;
  bool watchdog_command_;
  int mode_;
  rclcpp::Time last_command_time_;
  rclcpp::Time failure_recover_time_;
  bool reset_;
  
  // Parámetros del motor
  CurtisMotorParams motor_params_;
  
  // Estado del hardware
  bool interlock_;
  bool last_interlock_;
  int fault_code_;
  std::string fault_description_;
  
  // Interfaces de estado
  double hw_state_velocity_;       // Velocidad actual (m/s)
  double hw_state_current_rms_;    // Corriente RMS (A)
  double hw_state_battery_current_; // Corriente de batería (A)
  double hw_state_voltage_;        // Voltaje (V)
  double hw_state_bdi_percentage_; // Porcentaje BDI (%)
  double hw_state_motor_rpm_;      // RPM del motor
  double hw_state_controller_temp_; // Temperatura del controlador (°C)
  double hw_state_motor_temp_;     // Temperatura del motor (°C)
  
  // Interfaces de comando
  double hw_command_throttle_;     // Comando de acelerador (-100% a 100%)
  
  // Interfaz CAN
  std::unique_ptr<CanInterface> can_interface_ptr_;
  
  // Métodos auxiliares
  const char* getFaultString(int fault_code);
};

}  // namespace caddy_ai2_ros2_control_hardware_curtis_motor_driver

#endif  // CADDY_AI2_ROS2_CONTROL_HARDWARE_CURTIS_MOTOR_DRIVER__CURTIS_MOTOR_DRIVER_HPP_