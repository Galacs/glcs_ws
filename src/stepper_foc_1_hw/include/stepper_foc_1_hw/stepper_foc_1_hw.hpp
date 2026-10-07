#ifndef STEPPER_FOC_1_HW__STEPPER_FOC_1_HW_HPP_
#define STEPPER_FOC_1_HW__STEPPER_FOC_1_HW_HPP_

#include <cstdint>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace stepper_foc_1_hw
{
// Wire protocol. Keep in sync with the firmware's can_protocol.h.
namespace wire
{
constexpr uint8_t CMD_SET_TARGET = 0x00;
constexpr uint8_t CMD_STATE = 0x01;
constexpr uint8_t CMD_STATUS = 0x02;

constexpr uint32_t CAN_BASE = 0x200;
constexpr uint32_t CAN_MASK = 0x7E0;
constexpr uint32_t make_id(uint8_t node, uint8_t cmd) { return CAN_BASE | ((node & 7u) << 2) | (cmd & 3u); }
constexpr bool     id_is_ours(uint32_t id) { return (id & CAN_MASK) == CAN_BASE; }
constexpr uint8_t  id_node(uint32_t id)    { return (id >> 2) & 7u; }
constexpr uint8_t  id_cmd(uint32_t id)     { return id & 3u; }

constexpr uint8_t MODE_IDLE = 0;
constexpr uint8_t MODE_VELOCITY = 1;
constexpr uint8_t MODE_POSITION = 2;

constexpr uint8_t FLAG_ENABLE = 1u << 0;
constexpr uint8_t FLAG_CLEAR_FAULTS = 1u << 1;
constexpr uint8_t FLAG_SET_ZERO = 1u << 2;

constexpr uint8_t STATUS_ENABLED = 1u << 0;
constexpr uint8_t STATUS_FAULT_A = 1u << 1;
constexpr uint8_t STATUS_FAULT_B = 1u << 2;

#pragma pack(push, 1)
struct SetTarget { float target; int16_t limit; uint8_t mode; uint8_t flags; };
struct State     { float pos; float vel; };
struct Status    { uint8_t mode; uint8_t flags; uint16_t loop_us; float voltage_q; };
#pragma pack(pop)

static_assert(sizeof(SetTarget) == 8, "wire size");
static_assert(offsetof(SetTarget, target) == 0, "layout");
static_assert(offsetof(SetTarget, limit)  == 4, "layout");
static_assert(offsetof(SetTarget, mode)   == 6, "layout");
static_assert(offsetof(SetTarget, flags)  == 7, "layout");
static_assert(sizeof(State) == 8, "wire size");
static_assert(sizeof(Status) == 8, "wire size");
}  // namespace wire

class StepperFoc1Hw : public hardware_interface::SystemInterface
{
public:
  ~StepperFoc1Hw() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Axis {
    std::string name;
    uint8_t node_id{1};
    double max_velocity{10.0};   // rad/s, position-mode velocity limit
    double max_current{0.0};     // A, velocity-mode current limit; 0 = firmware default
    bool zero_on_activate{true};
    bool cmd_position{false};
    bool cmd_velocity{false};

    double position_command{0.0};
    double velocity_command{0.0};
    double position_state{0.0};
    double velocity_state{0.0};

    uint8_t mode{wire::MODE_IDLE};
    uint8_t fw_mode{wire::MODE_IDLE};   // last mode the node reported
    uint8_t pending_flags{0};           // one-shot flags, consumed by sendSetTarget()
    bool enabled{false};       // what we asked the node to do
    bool fw_enabled{false};    // what the node reports
    uint8_t faults{0};
    uint16_t loop_us{0};
    double voltage_q{0.0};

    bool has_state{false};
    bool position_primed{false};
    bool position_claimed{false};
    bool velocity_claimed{false};

    rclcpp::Time last_state_stamp{0, 0, RCL_STEADY_TIME};
    rclcpp::Time prime_not_before{0, 0, RCL_STEADY_TIME};
  };

  int command_timeout_ms_{0};
  int feedback_timeout_ms_{200};
  double zero_confirm_delay_s_{0.05};

  bool openSocket();
  void closeSocket();
  int  receiveFrame(uint32_t & can_id, uint8_t & len, uint8_t * data);
  bool sendFrame(uint8_t node_id, uint8_t cmd_id, const void * payload, uint8_t len);
  bool sendSetTarget(Axis & axis);
  void handleFrame(uint32_t can_id, uint8_t len, const uint8_t * data);
  void selectMode(Axis & axis);

  std::string can_interface_{"can0"};
  bool use_fd_{true};
  std::vector<Axis> axes_;
  int socket_fd_{-1};

  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

}  // namespace stepper_foc_1_hw

#endif  // STEPPER_FOC_1_HW__STEPPER_FOC_1_HW_HPP_