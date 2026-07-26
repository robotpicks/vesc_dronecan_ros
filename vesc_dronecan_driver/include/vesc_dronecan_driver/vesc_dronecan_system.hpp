#ifndef VESC_DRONECAN_DRIVER__VESC_DRONECAN_SYSTEM_HPP_
#define VESC_DRONECAN_DRIVER__VESC_DRONECAN_SYSTEM_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"

extern "C" {
#include "canard.h"
}

namespace vesc_dronecan_driver
{

// One drive wheel: a joint name (from URDF) paired with its DroneCAN
// uavcan.equipment.esc.{RPMCommand,Status} esc_index -- the value that VESC's uavcan_esc_index
// config field is set to (App Settings -> General -> UAVCAN ESC index in VESC Tool). Commanded in
// velocity, because VESC's RPMCommand handler routes to mc_interface_set_pid_speed(), a
// closed-loop speed PID. The older esc.RawCommand path was duty cycle, which could not have
// backed a velocity interface honestly.
struct DriveJoint
{
  std::string name;
  uint8_t esc_index = 0;
  double velocity_state = 0.0;   // rad/s at the wheel, from esc.Status
  double position_state = 0.0;   // rad, integrated from velocity -- esc.Status carries no position
  double velocity_command = 0.0; // rad/s at the wheel, sent as esc.RPMCommand
};

// One steering actuator: joint name paired with its uavcan.equipment.actuator.{ArrayCommand,
// Status} actuator_id. VESC has no separate actuator_id field -- the same uavcan_esc_index is
// reused as the actuator_id, so drive wheels and steering actuators share one id space and must
// be allocated distinct values across the bus (see the README's id-allocation section).
//
// Steering requires firmware that handles uavcan.equipment.actuator.ArrayCommand/Status. Upstream
// VESC firmware does NOT -- see the README's firmware-support section.
struct SteeringJoint
{
  std::string name;
  uint8_t actuator_id = 0;
  double position_state = 0.0;   // radians, from actuator.Status
  double velocity_state = 0.0;   // rad/s, from actuator.Status
  double position_command = 0.0; // radians, sent as actuator.ArrayCommand
};

// Per-ESC telemetry that has no natural joint interface: exported as <gpio> state interfaces so
// it reaches /dynamic_joint_states via joint_state_broadcaster, for whatever consumes pack
// telemetry downstream. Keyed to a drive wheel by the same esc_index.
//
// joint_state_broadcaster needs BOTH publish_dynamic_joint_states: true and
// use_urdf_to_filter: false for these to appear -- see the README, both default to suppressing
// them and neither failure is reported anywhere.
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
class VescDroneCanSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VescDroneCanSystem)

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

  // Motor revolutions per wheel revolution (gearbox and/or belt). 1.0 is direct drive.
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

}  // namespace vesc_dronecan_driver

#endif  // VESC_DRONECAN_DRIVER__VESC_DRONECAN_SYSTEM_HPP_
