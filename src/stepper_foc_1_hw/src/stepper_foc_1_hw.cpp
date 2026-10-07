#include "stepper_foc_1_hw/stepper_foc_1_hw.hpp"

#include <cerrno>
#include <cstring>
#include <algorithm>
#include <cmath>

#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace stepper_foc_1_hw
{
namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("StepperFoc1Hw"); }

template <typename MapT>
std::string getStringParam(const MapT & p, const std::string & k, const std::string & fb)
{
  auto it = p.find(k);
  return it != p.end() ? it->second : fb;
}

template <typename MapT>
int getIntParam(const MapT & p, const std::string & k, int fb)
{
  auto it = p.find(k);
  return it != p.end() ? std::stoi(it->second) : fb;
}

template <typename MapT>
double getDoubleParam(const MapT & p, const std::string & k, double fb)
{
  auto it = p.find(k);
  return it != p.end() ? std::stod(it->second) : fb;
}

template <typename MapT>
bool getBoolParam(const MapT & p, const std::string & k, bool fb)
{
  auto it = p.find(k);
  if (it == p.end()) return fb;
  return it->second == "true" || it->second == "1";
}
}  // namespace

StepperFoc1Hw::~StepperFoc1Hw() { closeSocket(); }

hardware_interface::CallbackReturn StepperFoc1Hw::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  can_interface_ = getStringParam(info_.hardware_parameters, "can_interface", can_interface_);
  use_fd_ = getBoolParam(info_.hardware_parameters, "use_fd", use_fd_);
  command_timeout_ms_ =
    getIntParam(info_.hardware_parameters, "command_timeout_ms", command_timeout_ms_);
  feedback_timeout_ms_ =
    getIntParam(info_.hardware_parameters, "feedback_timeout_ms", feedback_timeout_ms_);
  zero_confirm_delay_s_ =
    getIntParam(info_.hardware_parameters, "zero_confirm_delay_ms", 50) / 1000.0;
  if (zero_confirm_delay_s_ < 0.0) zero_confirm_delay_s_ = 0.0;

  for (const auto & joint : info_.joints)
  {
    Axis axis;
    axis.name = joint.name;
    axis.node_id = static_cast<uint8_t>(getIntParam(joint.parameters, "node_id", 1));
    axis.max_velocity = getDoubleParam(joint.parameters, "max_velocity", 10.0);
    axis.max_current = getDoubleParam(joint.parameters, "max_current", 0.0);
    axis.zero_on_activate =
      getBoolParam(joint.parameters, "zero_position_on_activate", true);

    if (axis.node_id == 0 || axis.node_id > 7)
    {
      RCLCPP_FATAL(logger(), "Joint '%s': node_id must be 1..7", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const auto & ci : joint.command_interfaces)
    {
      if (ci.name == hardware_interface::HW_IF_POSITION) axis.cmd_position = true;
      else if (ci.name == hardware_interface::HW_IF_VELOCITY) axis.cmd_velocity = true;
      else
      {
        RCLCPP_FATAL(
          logger(), "Joint '%s': unsupported command interface '%s' (only position and "
                    "velocity are implemented)", joint.name.c_str(), ci.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (!axis.cmd_position && !axis.cmd_velocity)
    {
      RCLCPP_FATAL(logger(), "Joint '%s': needs a position or velocity command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const auto & si : joint.state_interfaces)
    {
      if (si.name != hardware_interface::HW_IF_POSITION &&
          si.name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          logger(), "Joint '%s': unsupported state interface '%s'", joint.name.c_str(),
          si.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    axes_.push_back(axis);
  }

  if (axes_.empty())
  {
    RCLCPP_FATAL(logger(), "No joints configured");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StepperFoc1Hw::on_configure(const rclcpp_lifecycle::State &)
{
  for (auto & axis : axes_)
  {
    axis.pending_flags = 0;
    axis.fw_mode = wire::MODE_IDLE;
    axis.prime_not_before = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    axis.position_state = 0.0;
    axis.velocity_state = 0.0;
    axis.position_command = 0.0;
    axis.velocity_command = 0.0;
    axis.has_state = false;
    axis.position_primed = false;
    axis.enabled = false;
    axis.fw_enabled = false;
    axis.faults = 0;
    axis.mode = wire::MODE_IDLE;
    axis.pending_flags = 0;
    axis.position_claimed = false;
    axis.velocity_claimed = false;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> StepperFoc1Hw::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto & axis : axes_)
  {
    state_interfaces.emplace_back(
      axis.name, hardware_interface::HW_IF_POSITION, &axis.position_state);
    state_interfaces.emplace_back(
      axis.name, hardware_interface::HW_IF_VELOCITY, &axis.velocity_state);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> StepperFoc1Hw::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto & axis : axes_)
  {
    if (axis.cmd_position)
      command_interfaces.emplace_back(
        axis.name, hardware_interface::HW_IF_POSITION, &axis.position_command);
    if (axis.cmd_velocity)
      command_interfaces.emplace_back(
        axis.name, hardware_interface::HW_IF_VELOCITY, &axis.velocity_command);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn StepperFoc1Hw::on_activate(const rclcpp_lifecycle::State &)
{
  if (!openSocket())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & axis : axes_)
  {
    axis.enabled = false;
    axis.mode = wire::MODE_IDLE;
    axis.pending_flags = axis.zero_on_activate ? wire::FLAG_SET_ZERO : 0;
    axis.has_state = false;
    axis.position_primed = false;
    axis.prime_not_before = axis.zero_on_activate
      ? clock_.now() + rclcpp::Duration::from_seconds(zero_confirm_delay_s_)
      : rclcpp::Time(0, 0, RCL_STEADY_TIME);

    if (!sendSetTarget(axis))
    {
      RCLCPP_WARN(logger(), "Joint '%s': could not reach node %u on activation",
        axis.name.c_str(), axis.node_id);
    }
  }

  RCLCPP_INFO(logger(), "Listening on %s (%s frames)", can_interface_.c_str(),
    use_fd_ ? "CAN FD" : "classic CAN");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StepperFoc1Hw::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & axis : axes_)
  {
    axis.enabled = false;
    axis.mode = wire::MODE_IDLE;
    axis.pending_flags = 0;
    sendSetTarget(axis);   // mode idle, no enable flag -> firmware calls motor.disable()
    // If you would rather hold position than release the coils, send
    //   axis.mode = wire::MODE_VELOCITY; axis.flags = wire::FLAG_ENABLE;
    // with a zero velocity_command instead.
  }
  closeSocket();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type StepperFoc1Hw::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  const std::string pos_suffix = std::string("/") + hardware_interface::HW_IF_POSITION;
  const std::string vel_suffix = std::string("/") + hardware_interface::HW_IF_VELOCITY;

  for (auto & axis : axes_)
  {
    bool changed = false;
    for (const auto & key : stop_interfaces)
    {
      if (key == axis.name + pos_suffix) { axis.position_claimed = false; changed = true; }
      else if (key == axis.name + vel_suffix) { axis.velocity_claimed = false; changed = true; }
    }
    for (const auto & key : start_interfaces)
    {
      if (key == axis.name + pos_suffix) { axis.position_claimed = true; changed = true; }
      else if (key == axis.name + vel_suffix) { axis.velocity_claimed = true; changed = true; }
    }

    if (changed)
    {
      selectMode(axis);
      sendSetTarget(axis);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type StepperFoc1Hw::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (socket_fd_ < 0) return hardware_interface::return_type::ERROR;

  uint32_t can_id = 0;
  uint8_t len = 0;
  uint8_t data[64];
  const int res = [&] {
    int r = 0;
    while ((r = receiveFrame(can_id, len, data)) == 1) handleFrame(can_id, len, data);
    return r;
  }();
  if (res < 0)
  {
    // Transient socket error: keep the last known state and let the next cycle retry.
    return hardware_interface::return_type::OK;
  }

  const auto now = clock_.now();
  for (auto & axis : axes_)
  {
    if (!axis.has_state) continue;
    if (axis.has_state && axis.enabled && axis.fw_mode != axis.mode)
    {
      RCLCPP_WARN_THROTTLE(
        logger(), clock_, 1000, "Joint '%s' (node %u): requested mode %u, node reports %u",
        axis.name.c_str(), axis.node_id, axis.mode, axis.fw_mode);
    }
    if ((now - axis.last_state_stamp).seconds() * 1000.0 > feedback_timeout_ms_)
    {
      RCLCPP_WARN_THROTTLE(
        logger(), clock_, 1000, "Joint '%s' (node %u): no feedback for %d ms",
        axis.name.c_str(), axis.node_id, feedback_timeout_ms_);
      axis.has_state = false;
    }
    if (axis.faults & (wire::STATUS_FAULT_A | wire::STATUS_FAULT_B))
    {
      RCLCPP_ERROR_THROTTLE(
        logger(), clock_, 1000, "Joint '%s' (node %u): driver fault, status flags 0x%02X",
        axis.name.c_str(), axis.node_id, axis.faults);
    }
  }

  // Standard ros2_control fail-safe: notice that the controller stopped writing
  // and drop the axis. Left disabled for now - the firmware watchdog
  // (ENABLE_COMMAND_TIMEOUT) is the one that actually matters.
  // if (command_timeout_ms_ > 0) { ... }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type StepperFoc1Hw::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (socket_fd_ < 0) return hardware_interface::return_type::ERROR;

  for (auto & axis : axes_)
  {
    if (!axis.enabled) continue;
    if (!sendSetTarget(axis))
    {
      RCLCPP_WARN_THROTTLE(
        logger(), clock_, 1000, "Joint '%s': failed to send setpoint", axis.name.c_str());
    }
  }
  return hardware_interface::return_type::OK;
}

// ============================================================================
// Helpers
// ============================================================================

bool StepperFoc1Hw::openSocket()
{
  closeSocket();

  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0)
  {
    RCLCPP_FATAL(logger(), "socket(PF_CAN) failed: %s", std::strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
  {
    RCLCPP_FATAL(logger(), "Interface '%s' not found: %s", can_interface_.c_str(),
      std::strerror(errno));
    closeSocket();
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    RCLCPP_FATAL(logger(), "bind(%s) failed: %s", can_interface_.c_str(), std::strerror(errno));
    closeSocket();
    return false;
  }

  struct can_filter rfilter;
  rfilter.can_id   = wire::CAN_BASE;
  rfilter.can_mask = wire::CAN_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
  if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0)
  {
    RCLCPP_FATAL(logger(), "CAN_RAW_FILTER on '%s' failed: %s", can_interface_.c_str(),
      std::strerror(errno));
    closeSocket();
    return false;
  }

  if (use_fd_)
  {
    const int enable = 1;
    if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0)
    {
      RCLCPP_FATAL(logger(), "CAN_RAW_FD_FRAMES on '%s' failed: %s (is the link up with 'fd on'?)",
        can_interface_.c_str(), std::strerror(errno));
      closeSocket();
      return false;
    }
  }

  const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0)
  {
    RCLCPP_FATAL(logger(), "Could not set O_NONBLOCK: %s", std::strerror(errno));
    closeSocket();
    return false;
  }

  return true;
}

void StepperFoc1Hw::closeSocket()
{
  if (socket_fd_ >= 0)
  {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

// 1 = frame decoded, 0 = nothing pending, -1 = error
int StepperFoc1Hw::receiveFrame(uint32_t & can_id, uint8_t & len, uint8_t * data)
{
  if (use_fd_)
  {
    struct canfd_frame frame;
    const ssize_t n = ::recv(socket_fd_, &frame, sizeof(frame), 0);
    if (n < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
      if (errno == EINTR) return 0;
      RCLCPP_WARN_THROTTLE(logger(), clock_, 1000, "recv() failed: %s", std::strerror(errno));
      return -1;
    }
    if (n < static_cast<ssize_t>(sizeof(struct canfd_frame)))
    {
      // Short header only: no payload to decode.
      if (n < static_cast<ssize_t>(offsetof(struct canfd_frame, data))) return 0;
    }
    if (frame.len > 64) return 0;
    can_id = frame.can_id;
    len = frame.len;
    std::memcpy(data, frame.data, len);
    return 1;
  }

  struct can_frame frame;
  const ssize_t n = ::recv(socket_fd_, &frame, sizeof(frame), 0);
  if (n < 0)
  {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
    RCLCPP_WARN_THROTTLE(logger(), clock_, 1000, "recv() failed: %s", std::strerror(errno));
    return -1;
  }
  can_id = frame.can_id;
  len = frame.can_dlc;
  std::memcpy(data, frame.data, std::min<uint8_t>(len, 8));
  return 1;
}

bool StepperFoc1Hw::sendFrame(uint8_t node_id, uint8_t cmd_id, const void * payload, uint8_t len)
{
  if (socket_fd_ < 0 || len > 8) return false;

  const uint32_t can_id = wire::make_id(node_id, cmd_id);

  if (use_fd_)
  {
    struct canfd_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.len = len;
    frame.flags = 0;   // no CANFD_BRS: the whole frame runs at the nominal bit rate
    std::memcpy(frame.data, payload, len);
    const ssize_t n = ::write(socket_fd_, &frame, sizeof(frame));
    return n == static_cast<ssize_t>(sizeof(frame));
  }

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id;
  frame.can_dlc = len;
  std::memcpy(frame.data, payload, len);
  const ssize_t n = ::write(socket_fd_, &frame, sizeof(frame));
  return n == static_cast<ssize_t>(sizeof(frame));
}

bool StepperFoc1Hw::sendSetTarget(Axis & axis)
{
  wire::SetTarget msg;
  msg.target = 0.0f;
  msg.limit  = 0;
  msg.mode   = axis.enabled ? axis.mode : wire::MODE_IDLE;
  msg.flags  = static_cast<uint8_t>(axis.pending_flags |
                                    (axis.enabled ? wire::FLAG_ENABLE : 0));

  if (axis.enabled && axis.mode == wire::MODE_POSITION)
  {
    msg.target = static_cast<float>(axis.position_command);
    msg.limit = static_cast<int16_t>(std::lround(axis.max_velocity * 100.0));
  }
  else if (axis.enabled && axis.mode == wire::MODE_VELOCITY)
  {
    msg.target = static_cast<float>(axis.velocity_command);
    msg.limit = static_cast<int16_t>(std::lround(axis.max_current * 100.0));
  }

  const bool ok = sendFrame(axis.node_id, wire::CMD_SET_TARGET, &msg, sizeof(msg));

  if (ok)
  {
    if (msg.flags & wire::FLAG_SET_ZERO)
    {
      axis.prime_not_before =
        clock_.now() + rclcpp::Duration::from_seconds(zero_confirm_delay_s_);
    }
    axis.pending_flags = 0;   // one-shot flags are consumed
  }
  return ok;
}

void StepperFoc1Hw::handleFrame(uint32_t can_id, uint8_t len, const uint8_t * data)
{
  if (can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) return;   // standard data frames only
  if (!wire::id_is_ours(can_id)) return;

  const uint8_t node_id = wire::id_node(can_id);
  const uint8_t cmd = wire::id_cmd(can_id);

  for (auto & axis : axes_)
  {
    if (axis.node_id != node_id) continue;

    if (cmd == wire::CMD_STATE && len >= sizeof(wire::State))
    {
      if (!axis.position_primed && clock_.now() >= axis.prime_not_before)
      {
        axis.position_command = axis.position_state;
        axis.velocity_command = 0.0;
        axis.position_primed = true;
      }
      wire::State msg;
      std::memcpy(&msg, data, sizeof(msg));
      axis.position_state = msg.pos;
      axis.velocity_state = msg.vel;
      axis.has_state = true;
      axis.last_state_stamp = clock_.now();

      if (!axis.position_primed)
      {
        // Same idea as priming the CL57PR command from the driver's counter:
        // start the controller from where the axis actually is.
        axis.position_command = axis.position_state;
        axis.velocity_command = 0.0;
        axis.position_primed = true;
      }
    }
    else if (cmd == wire::CMD_STATUS && len >= sizeof(wire::Status))
    {
      wire::Status msg;
      std::memcpy(&msg, data, sizeof(msg));
      axis.fw_enabled = (msg.flags & wire::STATUS_ENABLED) != 0;
      axis.fw_mode = msg.mode;
      axis.faults = static_cast<uint8_t>(msg.flags & (wire::STATUS_FAULT_A | wire::STATUS_FAULT_B));
      axis.loop_us = msg.loop_us;
      axis.voltage_q = msg.voltage_q;
    }
    return;
  }
}

void StepperFoc1Hw::selectMode(Axis & axis)
{
  if (axis.position_claimed && axis.velocity_claimed)
  {
    RCLCPP_WARN(
      logger(), "Joint '%s': position and velocity are both claimed; running position. "
                "A SimpleFOC node has a single control mode, so only activate one "
                "controller per joint.", axis.name.c_str());
  }

  if (axis.position_claimed) axis.mode = wire::MODE_POSITION;
  else if (axis.velocity_claimed) axis.mode = wire::MODE_VELOCITY;
  else axis.mode = wire::MODE_IDLE;

  const bool wanted = (axis.mode != wire::MODE_IDLE);
  if (wanted && !axis.enabled)
  {
    RCLCPP_INFO(logger(), "Joint '%s': enabling node %u in %s mode", axis.name.c_str(),
      axis.node_id, axis.mode == wire::MODE_POSITION ? "position" : "velocity");
    axis.enabled = true;
    axis.pending_flags = static_cast<uint8_t>(axis.pending_flags | wire::FLAG_CLEAR_FAULTS);
  }
  else if (!wanted && axis.enabled)
  {
    RCLCPP_INFO(logger(), "Joint '%s': disabling node %u", axis.name.c_str(), axis.node_id);
    axis.enabled = false;
  }
}

}  // namespace stepper_foc_1_hw

PLUGINLIB_EXPORT_CLASS(stepper_foc_1_hw::StepperFoc1Hw, hardware_interface::SystemInterface)