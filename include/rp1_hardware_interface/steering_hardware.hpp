#ifndef RP1_HARDWARE_INTERFACE__STEERING_HARDWARE_HPP_
#define RP1_HARDWARE_INTERFACE__STEERING_HARDWARE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"

extern "C" {
#include "canard.h"
}

namespace rp1_hardware_interface
{

// One steering actuator: a joint name (from URDF) paired with its DroneCAN
// uavcan.equipment.actuator.{ArrayCommand,Status} actuator_id (the same value each VESC's
// uavcan_esc_index config field is set to -- see docs/can_id_map.md's +4 steering convention).
struct SteeringJoint
{
  std::string name;
  uint8_t actuator_id = 0;
  double position_state = 0.0;   // radians, from actuator.Status
  double velocity_state = 0.0;   // rad/s, from actuator.Status
  double position_command = 0.0; // radians, sent as actuator.ArrayCommand
};

// ros2_control SystemInterface talking uavcan.equipment.actuator.{ArrayCommand,Status} directly
// over SocketCAN (via a vendored copy of the same libcanard codec verified against
// /home/user/dev/bldc's firmware implementation -- see vendor/libcanard/README in this package).
class SteeringHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SteeringHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static void onTransferReceived(CanardInstance * ins, CanardRxTransfer * transfer);
  static bool shouldAcceptTransfer(
    const CanardInstance * ins, uint64_t * out_data_type_signature, uint16_t data_type_id,
    CanardTransferType transfer_type, uint8_t source_node_id);

  void handleActuatorStatus(CanardRxTransfer * transfer);
  void pumpTxQueue();
  void pumpRx();

  std::vector<SteeringJoint> joints_;

  std::string can_iface_ = "can0";
  uint8_t local_node_id_ = 44; // distinct from rp1_dronecan_bridge's default (42)
  int socket_fd_ = -1;

  CanardInstance canard_ins_{};
  std::vector<uint8_t> canard_memory_pool_;
  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

}  // namespace rp1_hardware_interface

#endif  // RP1_HARDWARE_INTERFACE__STEERING_HARDWARE_HPP_
