#ifndef RP1_HARDWARE_INTERFACE__RP1_HARDWARE_HPP_
#define RP1_HARDWARE_INTERFACE__RP1_HARDWARE_HPP_

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

// One drive wheel: a joint name (from URDF) paired with its DroneCAN
// uavcan.equipment.esc.{RPMCommand,Status} esc_index -- the value that VESC's uavcan_esc_index
// config field is set to (see docs/can_id_map.md). Commanded in velocity, because VESC's
// RPMCommand handler routes to mc_interface_set_pid_speed(), a closed-loop speed PID. The older
// esc.RawCommand path was duty cycle, which could not have backed a velocity interface honestly.
struct DriveJoint
{
  std::string name;
  uint8_t esc_index = 0;
  double velocity_state = 0.0;   // rad/s at the wheel, from esc.Status
  double position_state = 0.0;   // rad, integrated from velocity -- esc.Status carries no position
  double velocity_command = 0.0; // rad/s at the wheel, sent as esc.RPMCommand
};

// One steering actuator: joint name paired with its uavcan.equipment.actuator.{ArrayCommand,
// Status} actuator_id (the same VESC uavcan_esc_index field, reused as actuator_id -- see
// docs/can_id_map.md's "steering actuator_id = drive wheel index + 4" convention).
struct SteeringJoint
{
  std::string name;
  uint8_t actuator_id = 0;
  double position_state = 0.0;   // radians, from actuator.Status
  double velocity_state = 0.0;   // rad/s, from actuator.Status
  double position_command = 0.0; // radians, sent as actuator.ArrayCommand
};

// Per-ESC telemetry that has no natural joint interface: exported as <gpio> state interfaces so
// it reaches /dynamic_joint_states via joint_state_broadcaster (rp1_elrs turns it into the
// handset's BatteryState). Keyed to a drive wheel by the same esc_index.
struct EscTelemetry
{
  std::string name;
  uint8_t esc_index = 0;
  double voltage = 0.0;      // volts
  double current = 0.0;      // amps
  double temperature = 0.0;  // degrees Celsius (esc.Status carries kelvin)
};

// ros2_control SystemInterface speaking DroneCAN directly over SocketCAN via a vendored copy of
// the same libcanard codec the VESC firmware uses. Handles both wheel kinds on one bus with one
// node ID: drive wheels on esc.RPMCommand/Status, steering on actuator.ArrayCommand/Status.
// Which kind a joint is comes from its URDF parameters -- "esc_index" for drive, "actuator_id"
// for steering.
class Rp1Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Rp1Hardware)

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
  void handleEscStatus(CanardRxTransfer * transfer);
  void pumpTxQueue();
  void pumpRx();
  void broadcastRpmCommand();
  void broadcastActuatorCommand();

  // rad/s at the wheel -> the value to put in esc.RPMCommand, and back again for feedback.
  double wheelRadPerSecToCommandRpm(double rad_per_sec) const;
  double statusRpmToWheelRadPerSec(double status_rpm) const;

  std::vector<DriveJoint> drive_joints_;
  std::vector<SteeringJoint> steering_joints_;
  std::vector<EscTelemetry> esc_telemetry_;

  std::string can_iface_ = "can0";
  uint8_t local_node_id_ = 42;
  int socket_fd_ = -1;

  // Motor revolutions per wheel revolution (gearbox and/or belt). §3.4.5 of
  // docs/mechanical_request.md.
  double gear_ratio_ = 1.0;
  // Motor pole pairs = si_motor_poles / 2 in VESC's configuration.
  double motor_pole_pairs_ = 1.0;
  // VESC firmware is asymmetric about RPM units on the DroneCAN wire:
  //   canard_driver.c sendEscStatus():          status.rpm = get_rpm() / (poles/2)  -> mechanical
  //   canard_driver.c handle_esc_rpm_command(): set_pid_speed(rpm_val) with no scaling -> ERPM
  // So a value read back from Status cannot be commanded verbatim; it is off by pole pairs.
  // Default true matches the firmware as it stands. If the fork is fixed to scale the command
  // side too, set this false in the URDF and the extra factor drops out.
  bool command_rpm_is_erpm_ = true;

  CanardInstance canard_ins_{};
  std::vector<uint8_t> canard_memory_pool_;
  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

}  // namespace rp1_hardware_interface

#endif  // RP1_HARDWARE_INTERFACE__RP1_HARDWARE_HPP_
