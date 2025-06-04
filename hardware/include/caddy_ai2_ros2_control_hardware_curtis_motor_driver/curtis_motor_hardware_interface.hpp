#ifndef CURTIS_MOTOR_HARDWARE_INTERFACE_HPP_
#define CURTIS_MOTOR_HARDWARE_INTERFACE_HPP_

#include "caddy_ai2_ros2_control_hardware_curtis_motor_driver/curtis_motor_driver.hpp"
#include "caddy_ai2_ros2_control_hardware_curtis_motor_driver/socket_can_interface.hpp"

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
#define WHEEL_DIAMETER_M 0.5 // m
#define APROX_THROTTLE_TO_MPS ((M_PI * WHEEL_DIAMETER_M) * ( (MAX_RPM / 60.0) / GEAR_RATIO)) / SHRT_MAX // m/s per throttle value
#define RECOVER_TIME 5 // secs
#define WATCHDOG_TIMEOUT 0.2 // secs

namespace caddy_ai2_ros2_control_hardware_curtis_motor_driver
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

  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

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
  
  struct can_frame throttle_frame_; // Frame for sending throttle commands
  std::vector<struct can_frame> frames;

  // Hardware
  std::unique_ptr<SocketCANInterface> can_interface_;
  std::unique_ptr<CurtisMotorDriver> curtis_driver_;
};

}  // namespace caddy_ai2_ros2_control_hardware_curtis_motor_driver

#endif  // CURTIS_MOTOR_HARDWARE_INTERFACE_HPP_