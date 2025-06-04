#include "curtis_motor_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
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

  can_interface_name_ = info_.hardware_parameters["can_interface"];
  update_rate_hz_ = hardware_interface::stod(info_.hardware_parameters["update_rate"]);

  RCLCPP_INFO(
    get_logger(), "Initializing Curtis Motor Hardware Interface with CAN interface: %s",
    can_interface_name_.c_str());

  RCLCPP_INFO(
    get_logger(), "Update rate: %.2f Hz", update_rate_hz_);
    

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

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

  // Enviamos un mensaje de reset al controlador y esperamos a que se inicialice
  curtis_driver_->create_0x226_frame(&throttle_frame_, 0, true);
  can_interface_->write(throttle_frame_);
  std::this_thread::sleep_for(std::chrono::seconds(RECOVER_TIME)); // Wait for controller to initialize

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurtisMotorHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Send zero throttle command to stop the motor
  curtis_driver_->create_0x226_frame(&throttle_frame_, 0, false);
  can_interface_->write(throttle_frame_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CurtisMotorHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Send zero throttle command to stop the motor
  curtis_driver_->create_0x226_frame(&throttle_frame_, 0, false);
  can_interface_->write(throttle_frame_);
  
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type CurtisMotorHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // // Read CAN frames at 1000Hz
  frames.clear();
  int num_frames = can_interface_->read(frames, 0);  // Non-blocking read
  
  if (num_frames > 0) {
    // Process frames and update internal data
    std::vector<bool> processed = curtis_driver_->process_frames(frames);
    
    for(int i = 0; i < num_frames; ++i) {
      if (processed[i]) {
        if (frames[i].can_id == curtis_driver_->FRAME_227_) {

        }
        else if (frames[i].can_id == curtis_driver_->FRAME_1A6_) {

        }
        else if (frames[i].can_id == curtis_driver_->FRAME_2A6_) {
        
        }
      }
    }
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CurtisMotorHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{


  
  return hardware_interface::return_type::OK;
}

}  // namespace curtis_motor_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(curtis_motor_hardware::CurtisMotorHardwareInterface, hardware_interface::ActuatorInterface)