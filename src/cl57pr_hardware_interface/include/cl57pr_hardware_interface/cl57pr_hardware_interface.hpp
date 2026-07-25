#ifndef CL57PR_HARDWARE_INTERFACE__CL57PR_HARDWARE_INTERFACE_HPP_
#define CL57PR_HARDWARE_INTERFACE__CL57PR_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Forward-declare libmodbus's context type instead of including <modbus/modbus.h>
// here. That header #defines TRUE/FALSE macros for legacy portability, which
// clobber an enum value literally named TRUE inside hardware_interface's own
// hardware_info.hpp if modbus.h is included first. This declaration matches
// libmodbus's real one (`typedef struct _modbus modbus_t;`) exactly, so it's a
// compatible redeclaration rather than a conflicting one. The real header is
// only included in the .cpp, after these hardware_interface headers.
struct _modbus;
typedef struct _modbus modbus_t;

namespace cl57pr_hardware_interface
{

// ============================================================================
// CL57PR Modbus RTU register map (DM-PR series)
// Source: manufacturer register map, matches the vendor example script this
// package was built against.
// ============================================================================
namespace registers
{
// Status (Read-Only)
constexpr uint16_t DRIVER_MODEL = 0x0000;
constexpr uint16_t DRIVER_VERSION = 0x0001;
constexpr uint16_t NODE_NUMBER = 0x0002;
constexpr uint16_t DIP_STATUS = 0x0003;
constexpr uint16_t ERROR_CODE = 0x0006;
constexpr uint16_t MOTION_STATUS = 0x0007;
constexpr uint16_t INPUT_STATUS = 0x0008;
constexpr uint16_t OUTPUT_STATUS = 0x0009;
constexpr uint16_t CURRENT_POS_HIGH = 0x000A;
constexpr uint16_t CURRENT_POS_LOW = 0x000B;
constexpr uint16_t CURRENT_SPEED = 0x000C;

// Motion control parameters (Read/Write)
constexpr uint16_t START_SPEED = 0x0020;
constexpr uint16_t ACCEL_TIME = 0x0021;
constexpr uint16_t DECEL_TIME = 0x0022;
constexpr uint16_t POSITION_SPEED = 0x0023;
constexpr uint16_t TARGET_PULSES_HIGH = 0x0024;
constexpr uint16_t TARGET_PULSES_LOW = 0x0025;
constexpr uint16_t MOTION_COMMAND = 0x0027;
constexpr uint16_t AUX_COMMAND = 0x002D;

// Performance / current parameters
constexpr uint16_t MAX_CURRENT = 0x0102;
constexpr uint16_t CLOSED_LOOP_CURRENT_PCT = 0x0103;
constexpr uint16_t BASE_CURRENT_PCT = 0x0104;
constexpr uint16_t OPEN_LOOP_CURRENT_PCT = 0x0105;
constexpr uint16_t LOCK_CURRENT_PCT = 0x0106;
}  // namespace registers

// Motion command bits (register MOTION_COMMAND, 0x0027)
namespace motion_bits
{
constexpr uint16_t POS_START = 1 << 0;
constexpr uint16_t SPEED_START = 1 << 1;
constexpr uint16_t POS_ABSOLUTE = 1 << 2;
constexpr uint16_t MODE_SWITCH = 1 << 3;  // interrupt current motion with new command
constexpr uint16_t HOME_START = 1 << 4;
constexpr uint16_t STOP = 1 << 8;
constexpr uint16_t ESTOP = 1 << 9;
}  // namespace motion_bits

// Motion status bits (register MOTION_STATUS, 0x0007)
namespace status_bits
{
constexpr uint16_t IN_POSITION = 1 << 0;
constexpr uint16_t HOMING_DONE = 1 << 1;
constexpr uint16_t IS_MOVING = 1 << 2;
constexpr uint16_t HAS_ALARM = 1 << 3;
constexpr uint16_t MOTOR_RELEASED = 1 << 4;  // 0 = enabled, 1 = released
constexpr uint16_t POS_SOFT_LIMIT = 1 << 5;
constexpr uint16_t NEG_SOFT_LIMIT = 1 << 6;
}  // namespace status_bits

// Auxiliary commands (register AUX_COMMAND, 0x002D)
namespace aux_commands
{
constexpr uint16_t MOTOR_RELEASE = 0x0011;
constexpr uint16_t MOTOR_ENABLE = 0x0012;
constexpr uint16_t CLEAR_ALARM = 0x0021;
constexpr uint16_t CLEAR_POSITION = 0x0031;
constexpr uint16_t FACTORY_RESET = 0x0041;
constexpr uint16_t SAVE_PARAMETERS = 0x0042;
}  // namespace aux_commands

class Cl57prHardwareInterface : public hardware_interface::SystemInterface
{
public:
  Cl57prHardwareInterface() = default;
  ~Cl57prHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- helpers -------------------------------------------------------------
  bool writeSingleRegister(uint16_t reg, uint16_t value);
  bool writeDoubleRegister(uint16_t reg_high, uint16_t reg_low, int32_t value);
  bool readRegisterBlock(uint16_t start_reg, uint16_t count, uint16_t * dest);
  void logModbusError(const std::string & context);

  double pulsesToRadians(int32_t pulses) const;
  int32_t radiansToPulses(double radians) const;
  double rpmToRadPerSec(int16_t rpm) const;

  // --- configuration (from <hardware><param> tags in the URDF/ros2_control xacro) ---
  std::string serial_port_{"/dev/ttyACM0"};
  int baud_rate_{9600};
  int slave_id_{1};
  double steps_per_rev_{10000.0};   // encoder/microstep resolution -> pulses per motor revolution
  int max_speed_rpm_{120};
  int start_speed_rpm_{30};
  int accel_time_ms_{200};
  int decel_time_ms_{200};
  bool zero_position_on_activate_{true};
  double position_epsilon_rad_{0.0005};  // ignore command noise smaller than this

  // --- modbus connection -----------------------------------------------------
  modbus_t * modbus_ctx_{nullptr};
  bool connected_{false};

  // --- ros2_control state/command storage -------------------------------------
  double hw_position_state_{0.0};
  double hw_velocity_state_{0.0};
  double hw_position_command_{0.0};

  int32_t last_commanded_pulses_{0};
  bool has_commanded_once_{false};

  uint16_t last_status_word_{0};
  uint16_t last_error_code_{0};

  rclcpp::Clock clock_{RCL_STEADY_TIME};
  rclcpp::Time last_error_log_time_;
};

}  // namespace cl57pr_hardware_interface

#endif  // CL57PR_HARDWARE_INTERFACE__CL57PR_HARDWARE_INTERFACE_HPP_
