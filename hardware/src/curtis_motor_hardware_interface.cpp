#include "curtis_motor_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace curtis_motor_hardware
{

hardware_interface::CallbackReturn CurtisMotorHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // // Get parameters from URDF
  // can_interface_name_ = info_.hardware_parameters["can_interface"];
  // if (can_interface_name_.empty()) {
  //   can_interface_name_ = "can_motor_drv";
  // }
  
  // verbose_ = false;
  // if (info_.hardware_parameters.count("verbose") > 0) {
  //   verbose_ = (info_.hardware_parameters["verbose"] == "true");
  // }
  
  // control_rate_hz_ = 25.0;  // Default control rate: 25Hz
  // if (info_.hardware_parameters.count("control_rate_hz") > 0) {
  //   control_rate_hz_ = std::stod(info_.hardware_parameters["control_rate_hz"]);
  // }
  // control_period_ms_ = std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_hz_));
  
  // // Initialize state and command interfaces
  // hw_velocity_command_ = 0.0;
  // hw_velocity_state_ = 0.0;
  // hw_position_state_ = 0.0;
  
  // hw_current_rms_ = 0.0;
  // hw_battery_current_ = 0.0;
  // hw_keyswitch_voltage_ = 0.0;
  // hw_bdi_percentage_ = 0.0;
  // hw_controller_temp_ = 0.0;
  // hw_motor_temp_ = 0.0;
  
  // has_fresh_data_ = false;
  
  // // Check if the required interfaces are defined
  // if (info_.joints.size() != 1) {
  //   RCLCPP_ERROR(
  //     rclcpp::get_logger("CurtisMotorHardwareInterface"),
  //     "Expected exactly 1 joint in the hardware interface, got %zu", info_.joints.size());
  //   return hardware_interface::CallbackReturn::ERROR;
  // }
  
  // // Define state interfaces
  // for (const hardware_interface::ComponentInfo & joint : info_.joints)
  // {
  //   // Check command interfaces
  //   if (joint.command_interfaces.size() != 1 ||
  //       joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
  //   {
  //     RCLCPP_ERROR(
  //       rclcpp::get_logger("CurtisMotorHardwareInterface"),
  //       "Joint '%s' has %zu command interfaces, expected 1 (%s)",
  //       joint.name.c_str(), joint.command_interfaces.size(),
  //       hardware_interface::HW_IF_VELOCITY);
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }

  //   // Check state interfaces
  //   if (joint.state_interfaces.size() != 2 ||
  //       joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
  //       joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
  //   {
  //     RCLCPP_ERROR(
  //       rclcpp::get_logger("CurtisMotorHardwareInterface"),
  //       "Joint '%s' has %zu state interfaces, expected 2 (%s and %s)",
  //       joint.name.c_str(), joint.state_interfaces.size(),
  //       hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }
  // }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurtisMotorHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // // Initialize CAN interface and Curtis driver
  // can_interface_ = std::make_unique<SocketCANInterface>(can_interface_name_);
  // curtis_driver_ = std::make_unique<CurtisMotorDriver>(verbose_);
  
  // RCLCPP_INFO(
  //   rclcpp::get_logger("CurtisMotorHardwareInterface"),
  //   "Configured Curtis Motor Hardware Interface with CAN interface: %s", can_interface_name_.c_str());
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurtisMotorHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // // Clean up resources
  // if (can_interface_ && can_interface_->isInitialized()) {
  //   can_interface_->close();
  // }
  
  // can_interface_.reset();
  // curtis_driver_.reset();
  
  // RCLCPP_INFO(
  //   rclcpp::get_logger("CurtisMotorHardwareInterface"),
  //   "Cleaned up Curtis Motor Hardware Interface");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurtisMotorHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // // Initialize CAN interface
  // if (!can_interface_->init()) {
  //   RCLCPP_ERROR(
  //     rclcpp::get_logger("CurtisMotorHardwareInterface"),
  //     "Failed to initialize CAN interface: %s", can_interface_name_.c_str());
  //   return hardware_interface::CallbackReturn::ERROR;
  // }
  
  // // Register state and command interfaces
  // for (const hardware_interface::ComponentInfo & joint : info_.joints)
  // {
  //   // State interfaces
  //   state_interfaces.emplace_back(
  //     joint.name, hardware_interface::HW_IF_POSITION, &hw_position_state_);
  //   state_interfaces.emplace_back(
  //     joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_state_);
    
  //   // Command interfaces
  //   command_interfaces.emplace_back(
  //     joint.name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_command_);
  // }
  
  // // Initialize timing
  // last_control_update_time_ = std::chrono::steady_clock::now();
  
  // RCLCPP_INFO(
  //   rclcpp::get_logger("CurtisMotorHardwareInterface"),
  //   "Activated Curtis Motor Hardware Interface");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurtisMotorHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // // Send zero throttle command to stop the motor
  // struct can_frame stop_frame;
  // curtis_driver_->send_throttle_command(&stop_frame, 0, true);
  // can_interface_->write(stop_frame);
  
  // // Close CAN interface
  // if (can_interface_->isInitialized()) {
  //   can_interface_->close();
  // }
  
  // // Clear interfaces
  // command_interfaces.clear();
  // state_interfaces.clear();
  
  // RCLCPP_INFO(
  //   rclcpp::get_logger("CurtisMotorHardwareInterface"),
  //   "Deactivated Curtis Motor Hardware Interface");
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CurtisMotorHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // // Read CAN frames at 1000Hz
  // std::vector<struct can_frame> frames;
  // int num_frames = can_interface_->read(frames, 0);  // Non-blocking read
  
  // if (num_frames > 0) {
  //   // Process frames and update internal data
  //   std::vector<bool> processed = curtis_driver_->process_frames(frames);
    
  //   // Check if we have fresh data
  //   auto current_timestamp = curtis_driver_->get_last_update();
  //   if (current_timestamp > last_data_timestamp_) {
  //     last_data_timestamp_ = current_timestamp;
  //     has_fresh_data_ = true;
      
  //     // Update hardware interface state only when we have fresh data
  //     hw_velocity_state_ = curtis_driver_->get_speed();  // Speed in m/s
      
  //     // Integrate position from velocity (simple integration)
  //     // This is a simplification - in a real system you might want to use encoder data
  //     static auto last_integration_time = std::chrono::system_clock::now();
  //     auto now = std::chrono::system_clock::now();
  //     double dt = std::chrono::duration<double>(now - last_integration_time).count();
  //     hw_position_state_ += hw_velocity_state_ * dt;
  //     last_integration_time = now;
      
  //     // Update additional motor data
  //     hw_current_rms_ = curtis_driver_->get_current_rms();
  //     hw_battery_current_ = curtis_driver_->get_battery_current();
  //     hw_keyswitch_voltage_ = curtis_driver_->get_keyswitch_voltage();
  //     hw_bdi_percentage_ = curtis_driver_->get_bdi_percentage();
  //     hw_controller_temp_ = curtis_driver_->get_controller_temp();
  //     hw_motor_temp_ = curtis_driver_->get_motor_temp();
  //   }
  // }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurtisMotorHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // // Send control commands at the specified control rate (default: 25Hz)
  // auto now = std::chrono::steady_clock::now();
  // auto elapsed = now - last_control_update_time_;
  
  // if (elapsed >= control_period_ms_) {
  //   // Convert velocity command to throttle value (-100 to 100)
  //   // This is a simplified conversion - you may need to adjust based on your motor characteristics
  //   int throttle_value = static_cast<int>(hw_velocity_command_ * 100.0 / 5.0);  // Assuming 5 m/s is max speed
    
  //   // Clamp throttle value
  //   throttle_value = std::max(-100, std::min(100, throttle_value));
    
  //   // Send throttle command
  //   struct can_frame throttle_frame;
  //   curtis_driver_->send_throttle_command(&throttle_frame, throttle_value);
  //   can_interface_->write(throttle_frame);
    
  //   // Update last control time
  //   last_control_update_time_ = now;
    
  //   if (verbose_) {
  //     RCLCPP_INFO(
  //       rclcpp::get_logger("CurtisMotorHardwareInterface"),
  //       "Sent throttle command: %d", throttle_value);
  //   }
  // }
  
  return hardware_interface::return_type::OK;
}

}  // namespace curtis_motor_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(curtis_motor_hardware::CurtisMotorHardwareInterface, hardware_interface::ActuatorInterface)