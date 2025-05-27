#ifndef CURTIS_MOTOR_DRIVER_H
#define CURTIS_MOTOR_DRIVER_H

#include <linux/can.h>
#include <vector>
#include <cstdint>
#include <chrono>
#include <string>

#define RESET_CODE	0x4850
#define MOTOR_DIRECTION				-1	// +1 REGULAR
										// -1 INVERTED

#define CURTIS_SPEED_DIVISOR	10.0
#define KMH_TO_MPS				0.277777778

#define FRAME_226 0x226 // Throttle command frame
#define FRAME_227 0x227 // First
#define FRAME_327 0x327 // Second
#define FRAME_1A6 0x1A6 // Third
#define FRAME_2A6 0x2A6 // Fourth
#define FRAME_726 0x726 // When communication is not established

class CurtisMotorDriver {
public:
    CurtisMotorDriver(bool verbose = false);
    ~CurtisMotorDriver();

    // Process CAN frames and update internal data
    std::vector<bool> process_frames(const std::vector<struct can_frame>& frames);
    
    // Prepare a throttle command frame
    bool send_throttle_command(struct can_frame* frame, int throttle_value, bool reset = false);
    
    // Getters for motor data
    float get_current_rms() const { return current_rms_; }
    float get_battery_current() const { return battery_current_; }
    float get_keyswitch_voltage() const { return keyswitch_voltage_; }
    float get_bdi_percentage() const { return bdi_percentage_; }
    float get_motor_rpm() const { return motor_rpm_; }
    float get_speed() const { return speed_; }
    float get_controller_temp() const { return controller_temp_; }
    float get_motor_temp() const { return motor_temp_; }

    std::string getFaultString(int fault_code);
    
    // Get last update timestamp for any data
    std::chrono::system_clock::time_point get_last_update() const { return last_update_; }

private:
    bool verbose_; // Verbose mode for debugging

    // Process specific CAN IDs
    bool process_0x227_frame(const struct can_frame& frame);
    bool process_0x327_frame(const struct can_frame& frame);
    bool process_0x1A6_frame(const struct can_frame& frame);
    bool process_0x2A6_frame(const struct can_frame& frame);
    bool process_0x726_frame(const struct can_frame& frame);

    // Motor data
    bool interlock_;           // Interlock state
    bool mode_auto_;          // Automatic mode
    bool mode_manual_;        // Manual mode
    bool on_fault_;          // Fault state
    int32_t fault_code_;      // Fault code
    std::string fault_description_; // Fault description
    
    float current_rms_;          // Amps
    float battery_current_;      // Amps
    float keyswitch_voltage_;    // Volts
    float bdi_percentage_;       // Percentage
    int32_t motor_rpm_;          // RPM
    float speed_;                // km/h
    float controller_temp_;      // Celsius
    float motor_temp_;           // Celsius
    float desired_throttle_;     // Desired throttle percentage

    // Work modes
    int32_t MODE_THROTTLE=0;
    int32_t MODE_TWIST=1;
    
    // Timestamp of last data update
    std::chrono::system_clock::time_point last_update_0x226_frame_;
    std::chrono::system_clock::time_point last_update_0x227_frame_;
    std::chrono::system_clock::time_point last_update_0x327_frame_;
    std::chrono::system_clock::time_point last_update_0x1A6_frame_;
    std::chrono::system_clock::time_point last_update_0x2A6_frame_;
    std::chrono::system_clock::time_point last_update_0x726_frame_;
    std::chrono::system_clock::time_point last_update_;
};

#endif // CURTIS_MOTOR_DRIVER_H