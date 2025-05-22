#include "caddy_ai2_ros2_control_hardware_curtis_motor_driver/curtis_motor_driver.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace caddy_ai2_ros2_control_hardware_curtis_motor_driver
{

hardware_interface::CallbackReturn CurtisMotorDriver::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Inicializar valores por defecto
  hw_state_velocity_ = 0.0;
  hw_state_current_rms_ = 0.0;
  hw_state_battery_current_ = 0.0;
  hw_state_voltage_ = 0.0;
  hw_state_bdi_percentage_ = 0.0;
  hw_state_motor_rpm_ = 0.0;
  hw_state_controller_temp_ = 0.0;
  hw_state_motor_temp_ = 0.0;
  hw_command_throttle_ = 0.0;
  
  interlock_ = false;
  last_interlock_ = false;
  fault_code_ = 0;
  fault_description_ = "";
  reset_ = false;
  
  // Leer parámetros del hardware
  can_interface_ = info.hardware_parameters.count("can_interface") ?
    info.hardware_parameters.at("can_interface") : DEFAULT_CAN_PORT;
  
  watchdog_command_ = info.hardware_parameters.count("watchdog_command") ?
    (info.hardware_parameters.at("watchdog_command") == "true") : true;
  
  mode_ = info.hardware_parameters.count("mode") ?
    std::stoi(info.hardware_parameters.at("mode")) : 0;
  
  // Parámetros del motor
  motor_params_.wheel_diameter = info.hardware_parameters.count("wheel_diameter") ?
    std::stod(info.hardware_parameters.at("wheel_diameter")) : 0.5;
  
  motor_params_.max_rpm = info.hardware_parameters.count("max_rpm") ?
    std::stod(info.hardware_parameters.at("max_rpm")) : 4300.0;
  
  motor_params_.gear_ratio = info.hardware_parameters.count("gear_ratio") ?
    std::stod(info.hardware_parameters.at("gear_ratio")) : 16.0;
  
  motor_params_.max_v_auto = info.hardware_parameters.count("max_v_auto") ?
    std::stod(info.hardware_parameters.at("max_v_auto")) : 2.0;
  
  motor_params_.motor_direction = info.hardware_parameters.count("motor_direction") ?
    std::stoi(info.hardware_parameters.at("motor_direction")) : MOTOR_DIRECTION;
  
  // Calcular valores derivados
  motor_params_.max_rps = motor_params_.max_rpm / 60.0;
  motor_params_.max_v_mps = M_PI * motor_params_.wheel_diameter * motor_params_.max_rps / motor_params_.gear_ratio;
  
  RCLCPP_INFO(
    rclcpp::get_logger("CurtisMotorDriver"),
    "Initialized Curtis Motor Driver with parameters: "
    "can_interface: %s, watchdog_command: %s, mode: %d, "
    "wheel_diameter: %.2f, max_rpm: %.2f, gear_ratio: %.2f, max_v_auto: %.2f, "
    "motor_direction: %d, max_v_mps: %.2f",
    can_interface_.c_str(), watchdog_command_ ? "true" : "false", mode_,
    motor_params_.wheel_diameter, motor_params_.max_rpm, motor_params_.gear_ratio,
    motor_params_.max_v_auto, motor_params_.motor_direction, motor_params_.max_v_mps);
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurtisMotorDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Crear la interfaz CAN
  can_interface_ptr_ = std::make_unique<CanInterface>(
    can_interface_, rclcpp::get_logger("CurtisMotorDriver"));
  
  if (!can_interface_ptr_->init()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CurtisMotorDriver"),
      "Failed to initialize CAN interface on %s", can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("CurtisMotorDriver"),
    "Successfully configured Curtis Motor Driver");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurtisMotorDriver::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Cerrar la interfaz CAN
  if (can_interface_ptr_) {
    can_interface_ptr_->close();
    can_interface_ptr_.reset();
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("CurtisMotorDriver"),
    "Cleaned up Curtis Motor Driver");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurtisMotorDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reiniciar valores
  hw_command_throttle_ = 0.0;
  
  // Enviar comando de throttle cero para inicializar
  if (!can_interface_ptr_->sendThrottle(0.0)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CurtisMotorDriver"),
      "Failed to send initial throttle command");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("CurtisMotorDriver"),
    "Successfully activated Curtis Motor Driver");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurtisMotorDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Enviar comando de throttle cero para detener
  if (!can_interface_ptr_->sendThrottle(0.0)) {
    RCLCPP_WARN(
      rclcpp::get_logger("CurtisMotorDriver"),
      "Failed to send zero throttle command during deactivation");
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("CurtisMotorDriver"),
    "Successfully deactivated Curtis Motor Driver");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurtisMotorDriver::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Intentar enviar comando de throttle cero para detener en caso de error
  if (can_interface_ptr_) {
    can_interface_ptr_->sendThrottle(0.0);
  }
  
  RCLCPP_ERROR(
    rclcpp::get_logger("CurtisMotorDriver"),
    "Curtis Motor Driver entered error state");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurtisMotorDriver::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Enviar comando de throttle cero para detener
  if (can_interface_ptr_) {
    can_interface_ptr_->sendThrottle(0.0);
    can_interface_ptr_->close();
    can_interface_ptr_.reset();
  }
  
  RCLCPP_INFO(
    rclcpp::get_logger("CurtisMotorDriver"),
    "Curtis Motor Driver has been shut down");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CurtisMotorDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  // Exportar interfaces de estado
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocity_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.name, "current_rms", &hw_state_current_rms_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.name, "battery_current", &hw_state_battery_current_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.name, "voltage", &hw_state_voltage_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.name, "bdi_percentage", &hw_state_bdi_percentage_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.name, "motor_rpm", &hw_state_motor_rpm_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.name, hardware_interface::HW_IF_TEMPERATURE, &hw_state_controller_temp_));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    info_.name, "motor_temperature", &hw_state_motor_temp_));
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CurtisMotorDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  // Exportar interfaces de comando
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    info_.name, "throttle", &hw_command_throttle_));
  
  return command_interfaces;
}

hardware_interface::return_type CurtisMotorDriver::read()
{
  if (!can_interface_ptr_) {
    return hardware_interface::return_type::ERROR;
  }
  
  // Leer mensajes CAN
  double velocity, current, temperature;
  int fault_code;
  
  if (!can_interface_ptr_->readStatus(velocity, current, temperature, fault_code)) {
    RCLCPP_WARN(
      rclcpp::get_logger("CurtisMotorDriver"),
      "No CAN messages received");
    // No es un error crítico si no hay mensajes
  }
  
  // Actualizar interfaces de estado
  hw_state_velocity_ = velocity;
  hw_state_current_rms_ = current;
  hw_state_controller_temp_ = temperature;
  fault_code_ = fault_code;
  
  // Obtener datos adicionales
  hw_state_battery_current_ = can_interface_ptr_->getBatteryCurrent();
  hw_state_voltage_ = can_interface_ptr_->getKeyswitchVoltage();
  hw_state_bdi_percentage_ = can_interface_ptr_->getBDIPercentage();
  hw_state_motor_rpm_ = can_interface_ptr_->getMotorRPM();
  hw_state_motor_temp_ = can_interface_ptr_->getMotorTemp();
  
  // Actualizar estado de interlock
  last_interlock_ = interlock_;
  interlock_ = can_interface_ptr_->getInterlock();
  
  // Actualizar descripción de fallo
  if (fault_code_ != 0) {
    fault_description_ = getFaultString(fault_code_);
    RCLCPP_WARN(
      rclcpp::get_logger("CurtisMotorDriver"),
      "Fault detected: %s (code: %d)", fault_description_.c_str(), fault_code_);
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurtisMotorDriver::write()
{
  if (!can_interface_ptr_) {
    return hardware_interface::return_type::ERROR;
  }
  
  // Verificar watchdog
  auto current_time = rclcpp::Clock().now();
  
  if (watchdog_command_ && 
      (current_time - last_command_time_).seconds() > WATCHDOG_TIMEOUT) {
    hw_command_throttle_ = 0.0;
  }
  
  // Si hay un fallo, intentar resetear
  if (fault_code_ != 0 && reset_) {
    if (!can_interface_ptr_->sendThrottle(0.0, true)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CurtisMotorDriver"),
        "Failed to send reset command");
      return hardware_interface::return_type::ERROR;
    }
    reset_ = false;
    return hardware_interface::return_type::OK;
  }
  
  // Verificar interlock
  if (!interlock_) {
    if (hw_command_throttle_ != 0.0) {
      RCLCPP_WARN(
        rclcpp::get_logger("CurtisMotorDriver"),
        "Interlock not set. Setting throttle to 0");
      hw_command_throttle_ = 0.0;
    }
  } else if (interlock_ && !last_interlock_) {
    RCLCPP_INFO(
      rclcpp::get_logger("CurtisMotorDriver"),
      "Interlock recovered. Setting throttle to 0");
    hw_command_throttle_ = 0.0;
  }
  
  // Enviar comando de throttle
  if (!can_interface_ptr_->sendThrottle(hw_command_throttle_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CurtisMotorDriver"),
      "Failed to send throttle command: %.2f", hw_command_throttle_);
    return hardware_interface::return_type::ERROR;
  }
  
  last_command_time_ = current_time;
  return hardware_interface::return_type::OK;
}

const char* CurtisMotorDriver::getFaultString(int fault_code)
{
  switch (fault_code) {
    case 38: return "MAIN_CONTACTOR_WELDED";
    case 39: return "MAIN_CONTACTOR_DID_NOT_CLOSE";
    case 45: return "POT_LOW_OVERCURRENT";
    case 42: return "THROTTLE_WIPER_LOW";
    case 41: return "THROTTLE_WIPER_HIGH";
    case 44: return "POT2_WIPER_LOW";
    case 43: return "POT2_WIPER_HIGH";
    case 46: return "EEPROM_FAILURE";
    case 47: return "HPD_SEQUENCING";
    case 17: return "SEVERE_UNDERVOLTAGE";
    case 18: return "SEVERE_OVERVOLTAGE";
    case 23: return "UNDERVOLTAGE_CUTBACK";
    case 24: return "OVERVOLTAGE_CUTBACK";
    case 22: return "CONTROLLER_OVERTEMP_CUTBACK";
    case 15: return "CONTROLLER_SEVERE_UNDERTEMP";
    case 16: return "CONTROLLER_SEVERE_OVERTEMP";
    case 33: return "COIL3_DRIVER_OPEN_SHORT";
    case 34: return "COIL4_DRIVER_OPEN_SHORT";
    case 35: return "PD_OPEN_SHORT";
    case 31: return "MAIN_OPEN_SHORT";
    case 32: return "EMBRAKE_OPEN_SHORT";
    case 14: return "PRECHARGE_FAILED";
    case 26: return "DIGITAL_OUT_6_OVERCURRENT";
    case 27: return "DIGITAL_OUT_7_OVERCURRENT";
    case 28: return "MOTOR_TEMP_HOT_CUTBACK";
    case 49: return "PARAMETER_CHANGE";
    case 37: return "MOTOR_OPEN";
    case 51: return "SLAVE_TIMEOUT_FAULT";
    case 52: return "CUSTOM_HPD_FAULT";
    case 53: return "SEQUENCING_FAULT";
    case 54: return "SLAVE_FAULTED";
    case 55: return "COMPUTER_TIMEOUT_FAULT";
    case 69: return "EXTERNAL_SUPPLY_OUT_OF_RANGE";
    case 29: return "MOTOR_TEMP_SENSOR";
    case 68: return "VCL_RUN_TIME_ERROR";
    case 25: return "EXTERNAL_5V";
    case 71: return "OS_GENERAL";
    case 72: return "PDO_TIMEOUT";
    case 36: return "ENCODER";
    case 73: return "STALL_DETECTED";
    case 89: return "MOTOR_TYPE_ERROR";
    case 87: return "MOTOR_CHARACTERIZATION";
    case 97: return "PUMP_HARDWARE";
    case 91: return "VCL_OS_MISMATCH";
    case 92: return "EM_BRAKE_FAILED_TO_SET";
    case 93: return "ENCODER_LOS";
    case 94: return "EMER_REV_TIMEOUT";
    case 75: return "DUAL_SEVERE";
    case 74: return "FAULT_ON_OTHER_TRACTION_CONTROLLER";
    case 98: return "ILLEGAL_MODEL_NUMBER";
    case 95: return "PUMP_OVERCURRENT";
    case 96: return "PUMP_BDI";
    case 99: return "DUALMOTOR_PARAMETER_MISMATCH";
    default: return "UNKNOWN_FAULT";
  }
}

}  // namespace caddy_ai2_ros2_control_hardware_curtis_motor_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  caddy_ai2_ros2_control_hardware_curtis_motor_driver::CurtisMotorDriver,
  hardware_interface::ActuatorInterface)