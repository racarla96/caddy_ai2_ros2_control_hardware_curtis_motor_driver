#ifndef CURTIS_MOTOR_HARDWARE_INTERFACE_HPP_
#define CURTIS_MOTOR_HARDWARE_INTERFACE_HPP_

#include "curtis_motor_driver.hpp"
#include "socket_can_interface.hpp"

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#define MAX_RPM 4300.0
#define GEAR_RATIO 16.0

namespace curtis_motor_hardware
{
class CurtisMotorHardwareInterface : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CurtisMotorHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters
  std::string can_interface_name_;
  bool verbose_;

  // Max rate for reading and writing data 25 Hz
  double update_rate_hz_; // Rate of the controller manager for read and write operations
  double control_rate_hz_; // Rate of the control loop for sending commands
  int control_rate_write_counts_; // Number of write operations per control loop iteration
  int control_rate_write_counter_; // Counter for control rate (update_rate_hz_ / control_rate_hz_)
  
  // Hardware
  std::unique_ptr<SocketCANInterface> can_interface_;
  std::unique_ptr<CurtisMotorDriver> curtis_driver_;
};

}  // namespace curtis_motor_hardware

#endif  // CURTIS_MOTOR_HARDWARE_INTERFACE_HPP_