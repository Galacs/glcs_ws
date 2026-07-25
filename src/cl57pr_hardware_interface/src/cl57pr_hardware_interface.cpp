#include "cl57pr_hardware_interface/cl57pr_hardware_interface.hpp"

#include <cmath>
#include <memory>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <modbus/modbus.h>

namespace cl57pr_hardware_interface
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
rclcpp::Logger logger() { return rclcpp::get_logger("Cl57prHardwareInterface"); }

// Read a parameter from HardwareInfo, falling back to a default if not present.
template <typename T>
T getParam(const hardware_interface::HardwareInfo & info, const std::string & key, T default_value);

template <>
std::string getParam<std::string>(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  std::string default_value)
{
  auto it = info.hardware_parameters.find(key);
  return it != info.hardware_parameters.end() ? it->second : default_value;
}

template <>
double getParam<double>(
  const hardware_interface::HardwareInfo & info, const std::string & key, double default_value)
{
  auto it = info.hardware_parameters.find(key);
  return it != info.hardware_parameters.end() ? std::stod(it->second) : default_value;
}

template <>
int getParam<int>(
  const hardware_interface::HardwareInfo & info, const std::string & key, int default_value)
{
  auto it = info.hardware_parameters.find(key);
  return it != info.hardware_parameters.end() ? std::stoi(it->second) : default_value;
}

template <>
bool getParam<bool>(
  const hardware_interface::HardwareInfo & info, const std::string & key, bool default_value)
{
  auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) return default_value;
  return it->second == "true" || it->second == "1";
}
}  // namespace

Cl57prHardwareInterface::~Cl57prHardwareInterface()
{
  if (modbus_ctx_)
  {
    modbus_close(modbus_ctx_);
    modbus_free(modbus_ctx_);
  }
}

hardware_interface::CallbackReturn Cl57prHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1)
  {
    RCLCPP_FATAL(
      logger(), "Cl57prHardwareInterface supports exactly one joint, got %zu",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & joint = info_.joints[0];

  if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_FATAL(
      logger(), "Joint '%s' must expose exactly one 'position' command interface",
      joint.name.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = getParam<std::string>(info_, "serial_port", serial_port_);
  baud_rate_ = getParam<int>(info_, "baud_rate", baud_rate_);
  slave_id_ = getParam<int>(info_, "slave_id", slave_id_);
  steps_per_rev_ = getParam<double>(info_, "steps_per_rev", steps_per_rev_);
  max_speed_rpm_ = getParam<int>(info_, "max_speed_rpm", max_speed_rpm_);
  start_speed_rpm_ = getParam<int>(info_, "start_speed_rpm", start_speed_rpm_);
  accel_time_ms_ = getParam<int>(info_, "accel_time_ms", accel_time_ms_);
  decel_time_ms_ = getParam<int>(info_, "decel_time_ms", decel_time_ms_);
  zero_position_on_activate_ =
    getParam<bool>(info_, "zero_position_on_activate", zero_position_on_activate_);
  position_epsilon_rad_ = getParam<double>(info_, "position_epsilon_rad", position_epsilon_rad_);

  if (steps_per_rev_ <= 0.0)
  {
    RCLCPP_FATAL(logger(), "steps_per_rev must be > 0 (got %f)", steps_per_rev_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Cl57prHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_position_state_ = 0.0;
  hw_velocity_state_ = 0.0;
  hw_position_command_ = 0.0;
  has_commanded_once_ = false;
  last_commanded_pulses_ = 0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Cl57prHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  const auto & joint_name = info_.joints[0].name;
  state_interfaces.emplace_back(
    joint_name, hardware_interface::HW_IF_POSITION, &hw_position_state_);
  state_interfaces.emplace_back(
    joint_name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_state_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Cl57prHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  const auto & joint_name = info_.joints[0].name;
  command_interfaces.emplace_back(
    joint_name, hardware_interface::HW_IF_POSITION, &hw_position_command_);
  return command_interfaces;
}

hardware_interface::CallbackReturn Cl57prHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  modbus_ctx_ = modbus_new_rtu(serial_port_.c_str(), baud_rate_, 'N', 8, 1);
  if (!modbus_ctx_)
  {
    RCLCPP_FATAL(
      logger(), "modbus_new_rtu failed for port '%s': %s", serial_port_.c_str(),
      modbus_strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (modbus_set_slave(modbus_ctx_, slave_id_) == -1)
  {
    RCLCPP_FATAL(logger(), "modbus_set_slave(%d) failed: %s", slave_id_, modbus_strerror(errno));
    modbus_free(modbus_ctx_);
    modbus_ctx_ = nullptr;
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (modbus_connect(modbus_ctx_) == -1)
  {
    RCLCPP_FATAL(
      logger(), "modbus_connect failed on '%s': %s", serial_port_.c_str(),
      modbus_strerror(errno));
    modbus_free(modbus_ctx_);
    modbus_ctx_ = nullptr;
    return hardware_interface::CallbackReturn::ERROR;
  }
  connected_ = true;

  // Response timeout: keep it short since this loop runs at control-loop rate.
  modbus_set_response_timeout(modbus_ctx_, 0, 50000);  // 50 ms

  RCLCPP_INFO(
    logger(), "Connected to CL57PR on %s @ %d baud, slave id %d", serial_port_.c_str(),
    baud_rate_, slave_id_);

  // Clear any latched alarm and enable the motor before commanding motion.
  if (!writeSingleRegister(registers::AUX_COMMAND, aux_commands::CLEAR_ALARM))
  {
    RCLCPP_WARN(logger(), "Failed to clear alarm on activate (may be none pending)");
  }
  if (!writeSingleRegister(registers::AUX_COMMAND, aux_commands::MOTOR_ENABLE))
  {
    RCLCPP_FATAL(logger(), "Failed to enable motor on activate");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load the (static) motion profile: start speed, accel/decel time, cruise speed.
  // Only the target-pulse registers are rewritten per control cycle.
  bool profile_ok = true;
  profile_ok &= writeSingleRegister(registers::START_SPEED, static_cast<uint16_t>(start_speed_rpm_));
  profile_ok &= writeSingleRegister(registers::ACCEL_TIME, static_cast<uint16_t>(accel_time_ms_));
  profile_ok &= writeSingleRegister(registers::DECEL_TIME, static_cast<uint16_t>(decel_time_ms_));
  profile_ok &= writeSingleRegister(registers::POSITION_SPEED, static_cast<uint16_t>(max_speed_rpm_));
  if (!profile_ok)
  {
    RCLCPP_FATAL(logger(), "Failed to write motion profile registers on activate");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (zero_position_on_activate_)
  {
    if (!writeSingleRegister(registers::AUX_COMMAND, aux_commands::CLEAR_POSITION))
    {
      RCLCPP_WARN(logger(), "Failed to clear position counter on activate");
    }
  }

  // Prime state and command with the driver's current actual position so the
  // controller does not see a jump on the first read/write cycle.
  uint16_t block[7] = {0};
  if (readRegisterBlock(registers::ERROR_CODE, 7, block))
  {
    const int32_t pos_pulses =
      (static_cast<int32_t>(block[4]) << 16) | static_cast<int32_t>(block[5]);
    hw_position_state_ = pulsesToRadians(pos_pulses);
    hw_position_command_ = hw_position_state_;
    last_commanded_pulses_ = pos_pulses;
  }
  has_commanded_once_ = false;

  last_error_log_time_ = clock_.now();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Cl57prHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (connected_)
  {
    // Decelerate to a stop rather than an emergency stop, then release the motor.
    writeSingleRegister(registers::MOTION_COMMAND, motion_bits::STOP);
    writeSingleRegister(registers::AUX_COMMAND, aux_commands::MOTOR_RELEASE);
    modbus_close(modbus_ctx_);
    connected_ = false;
  }
  if (modbus_ctx_)
  {
    modbus_free(modbus_ctx_);
    modbus_ctx_ = nullptr;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Cl57prHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!connected_) return hardware_interface::return_type::ERROR;

  // ERROR_CODE(0x0006) .. CURRENT_SPEED(0x000C) is one contiguous block of 7 registers:
  // [error, motion_status, input_status, output_status, pos_high, pos_low, speed]
  uint16_t block[7] = {0};
  if (!readRegisterBlock(registers::ERROR_CODE, 7, block))
  {
    logModbusError("read status/position block");
    return hardware_interface::return_type::OK;  // keep last known state, don't halt the loop
  }

  last_error_code_ = block[0];
  last_status_word_ = block[1];
  const int32_t pos_pulses = (static_cast<int32_t>(block[4]) << 16) | static_cast<int32_t>(block[5]);
  const int16_t speed_rpm = static_cast<int16_t>(block[6]);

  hw_position_state_ = pulsesToRadians(pos_pulses);
  hw_velocity_state_ = rpmToRadPerSec(speed_rpm);

  if (last_status_word_ & status_bits::HAS_ALARM)
  {
    auto now = clock_.now();
    if ((now - last_error_log_time_).seconds() > 1.0)
    {
      RCLCPP_ERROR(
        logger(), "CL57PR alarm active, error code 0x%04X", last_error_code_);
      last_error_log_time_ = now;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Cl57prHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!connected_) return hardware_interface::return_type::ERROR;

  // Don't fight the driver while it's latched into an alarm state; a clear_alarm
  // service/param hook would go here in a fuller implementation.
  if (last_status_word_ & status_bits::HAS_ALARM)
  {
    return hardware_interface::return_type::OK;
  }

  if (
    has_commanded_once_ &&
    std::abs(hw_position_command_ - pulsesToRadians(last_commanded_pulses_)) <
      position_epsilon_rad_)
  {
    return hardware_interface::return_type::OK;
  }

  const int32_t target_pulses = radiansToPulses(hw_position_command_);

  if (!writeDoubleRegister(registers::TARGET_PULSES_HIGH, registers::TARGET_PULSES_LOW, target_pulses))
  {
    logModbusError("write target pulses");
    return hardware_interface::return_type::OK;
  }

  // Absolute positioning move, allowed to interrupt any motion already in progress.
  const uint16_t command =
    motion_bits::POS_START | motion_bits::POS_ABSOLUTE | motion_bits::MODE_SWITCH;
  if (!writeSingleRegister(registers::MOTION_COMMAND, command))
  {
    logModbusError("write motion command");
    return hardware_interface::return_type::OK;
  }

  last_commanded_pulses_ = target_pulses;
  has_commanded_once_ = true;

  return hardware_interface::return_type::OK;
}

// ============================================================================
// Helpers
// ============================================================================

bool Cl57prHardwareInterface::writeSingleRegister(uint16_t reg, uint16_t value)
{
  return modbus_write_register(modbus_ctx_, reg, value) != -1;
}

bool Cl57prHardwareInterface::writeDoubleRegister(uint16_t reg_high, uint16_t reg_low, int32_t value)
{
  uint16_t regs[2];
  regs[0] = static_cast<uint16_t>((static_cast<uint32_t>(value) >> 16) & 0xFFFF);
  regs[1] = static_cast<uint16_t>(static_cast<uint32_t>(value) & 0xFFFF);
  // reg_low is always reg_high + 1 in this register map; write both in one transaction.
  (void)reg_low;
  return modbus_write_registers(modbus_ctx_, reg_high, 2, regs) != -1;
}

bool Cl57prHardwareInterface::readRegisterBlock(uint16_t start_reg, uint16_t count, uint16_t * dest)
{
  return modbus_read_registers(modbus_ctx_, start_reg, count, dest) != -1;
}

void Cl57prHardwareInterface::logModbusError(const std::string & context)
{
  RCLCPP_WARN_THROTTLE(
    logger(), clock_, 1000, "Modbus error during %s: %s", context.c_str(), modbus_strerror(errno));
}

double Cl57prHardwareInterface::pulsesToRadians(int32_t pulses) const
{
  return static_cast<double>(pulses) / steps_per_rev_ * kTwoPi;
}

int32_t Cl57prHardwareInterface::radiansToPulses(double radians) const
{
  return static_cast<int32_t>(std::lround(radians / kTwoPi * steps_per_rev_));
}

double Cl57prHardwareInterface::rpmToRadPerSec(int16_t rpm) const
{
  return static_cast<double>(rpm) * kTwoPi / 60.0;
}

}  // namespace cl57pr_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  cl57pr_hardware_interface::Cl57prHardwareInterface, hardware_interface::SystemInterface)
