#include "vesc_dronecan_driver/vesc_dronecan_system.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

extern "C" {
#include "uavcan/equipment/actuator/ArrayCommand.h"
#include "uavcan/equipment/actuator/Command.h"
#include "uavcan/equipment/actuator/Status.h"
#include "uavcan/equipment/esc/RPMCommand.h"
#include "uavcan/equipment/esc/Status.h"
}

namespace vesc_dronecan_driver
{

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("vesc_dronecan_driver"); }

constexpr double kRadPerSecToRpm = 60.0 / (2.0 * M_PI);
constexpr double kRpmToRadPerSec = (2.0 * M_PI) / 60.0;
constexpr double kKelvinOffset = 273.15;

// esc.Status.rpm is an 18-bit signed field; RPMCommand's array elements likewise.
constexpr double kEscRpmMax = 131071.0;

double parseDouble(const std::string & text, double fallback)
{
  try {
    return std::stod(text);
  } catch (const std::exception &) {
    return fallback;
  }
}
}  // namespace

hardware_interface::CallbackReturn VescDroneCanSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // Base on_init() parses the URDF-declared state/command interfaces. Current ros2_control passes
  // a HardwareComponentInterfaceParams (HardwareInfo plus a weak_ptr to the controller manager's
  // executor); the old HardwareInfo-only overload is gone, so the URDF data is reached through
  // params.hardware_info.
  auto base_result = hardware_interface::SystemInterface::on_init(params);
  if (base_result != hardware_interface::CallbackReturn::SUCCESS) {
    return base_result;
  }

  const auto & info = params.hardware_info;

  auto param = [&info](const std::string & key) -> const std::string * {
    auto it = info.hardware_parameters.find(key);
    return it == info.hardware_parameters.end() ? nullptr : &it->second;
  };

  if (auto * v = param("can_iface")) {
    can_iface_ = *v;
  }
  if (auto * v = param("node_id")) {
    local_node_id_ = static_cast<uint8_t>(std::stoi(*v));
  }
  if (auto * v = param("gear_ratio")) {
    gear_ratio_ = parseDouble(*v, gear_ratio_);
  }
  if (auto * v = param("motor_pole_pairs")) {
    motor_pole_pairs_ = parseDouble(*v, motor_pole_pairs_);
  }
  if (auto * v = param("command_rpm_is_erpm")) {
    command_rpm_is_erpm_ = (*v == "true" || *v == "True" || *v == "1");
  }
  if (auto * v = param("esc_timeout_sec")) {
    esc_timeout_sec_ = parseDouble(*v, esc_timeout_sec_);
  }

  if (gear_ratio_ <= 0.0 || motor_pole_pairs_ <= 0.0 || esc_timeout_sec_ <= 0.0) {
    RCLCPP_ERROR(
      logger(), "gear_ratio (%f), motor_pole_pairs (%f) and esc_timeout_sec (%f) must all be "
      "positive", gear_ratio_, motor_pole_pairs_, esc_timeout_sec_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  drive_joints_.clear();
  steering_joints_.clear();
  for (const auto & joint : info.joints) {
    const bool has_esc = joint.parameters.count("esc_index") > 0;
    const bool has_actuator = joint.parameters.count("actuator_id") > 0;

    if (has_esc == has_actuator) {
      RCLCPP_ERROR(
        logger(),
        "Joint '%s' must declare exactly one of 'esc_index' (drive wheel) or 'actuator_id' "
        "(steering actuator); it declares %s",
        joint.name.c_str(), has_esc ? "both" : "neither");
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (has_esc) {
      DriveJoint drive;
      drive.name = joint.name;
      drive.esc_index = static_cast<uint8_t>(std::stoi(joint.parameters.at("esc_index")));
      drive_joints_.push_back(drive);
    } else {
      SteeringJoint steering;
      steering.name = joint.name;
      steering.actuator_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("actuator_id")));
      steering_joints_.push_back(steering);
    }
  }

  // <gpio> blocks carry the per-ESC telemetry and per-steering-actuator sensor status that have
  // no joint interface of their own -- distinguished by which id parameter they declare, the
  // same esc_index/actuator_id split the joints above use.
  esc_telemetry_.clear();
  steering_sensors_.clear();
  for (const auto & gpio : info.gpios) {
    auto esc_it = gpio.parameters.find("esc_index");
    auto actuator_it = gpio.parameters.find("actuator_id");
    if (esc_it != gpio.parameters.end() == (actuator_it != gpio.parameters.end())) {
      RCLCPP_ERROR(
        logger(), "GPIO '%s' must declare exactly one of 'esc_index' or 'actuator_id'",
        gpio.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (esc_it != gpio.parameters.end()) {
      EscTelemetry telemetry;
      telemetry.name = gpio.name;
      telemetry.esc_index = static_cast<uint8_t>(std::stoi(esc_it->second));
      esc_telemetry_.push_back(telemetry);
    } else {
      SteeringSensors sensors;
      sensors.name = gpio.name;
      sensors.actuator_id = static_cast<uint8_t>(std::stoi(actuator_it->second));
      steering_sensors_.push_back(sensors);
    }
  }

  canard_memory_pool_.resize(4096);

  RCLCPP_INFO(
    logger(),
    "vesc_dronecan_driver configured: %zu drive, %zu steering, %zu ESC telemetry, "
    "%zu steering sensors",
    drive_joints_.size(), steering_joints_.size(), esc_telemetry_.size(),
    steering_sensors_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescDroneCanSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset the esc-presence watchdog so a timestamp from a previous activation can't linger and
  // make a currently-missing ESC read as "recently seen".
  for (auto & joint : drive_joints_) {
    joint.last_status_time = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  }

  canardInit(
    &canard_ins_, canard_memory_pool_.data(), canard_memory_pool_.size(), &onTransferReceived,
    &shouldAcceptTransfer, this);
  canardSetLocalNodeID(&canard_ins_, local_node_id_);

  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(logger(), "Failed to open SocketCAN socket: %s", std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, can_iface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_ERROR(
      logger(), "Failed to find CAN interface '%s': %s", can_iface_.c_str(), std::strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(
      logger(), "Failed to bind to CAN interface '%s': %s", can_iface_.c_str(),
      std::strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  int flags = fcntl(socket_fd_, F_GETFL, 0);
  fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

  RCLCPP_INFO(
    logger(),
    "vesc_dronecan_driver up on %s (node_id=%d, %zu drive + %zu steering joint(s), "
    "gear_ratio=%.4f, pole_pairs=%.1f, command_rpm_is_erpm=%s, esc_timeout_sec=%.2f)",
    can_iface_.c_str(), local_node_id_, drive_joints_.size(), steering_joints_.size(), gear_ratio_,
    motor_pole_pairs_, command_rpm_is_erpm_ ? "true" : "false", esc_timeout_sec_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescDroneCanSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

double VescDroneCanSystem::wheelRadPerSecToCommandRpm(double rad_per_sec) const
{
  // wheel rad/s -> wheel RPM -> motor RPM, then (because the firmware feeds RPMCommand straight
  // into mc_interface_set_pid_speed without a pole-pair conversion) motor RPM -> ERPM.
  double rpm = rad_per_sec * kRadPerSecToRpm * gear_ratio_;
  if (command_rpm_is_erpm_) {
    rpm *= motor_pole_pairs_;
  }
  return rpm;
}

double VescDroneCanSystem::statusRpmToWheelRadPerSec(double status_rpm) const
{
  // esc.Status.rpm is already mechanical motor RPM -- sendEscStatus() divides ERPM by pole pairs
  // before transmitting -- so only the gearing has to come back out here.
  return (status_rpm / gear_ratio_) * kRpmToRadPerSec;
}

void VescDroneCanSystem::pumpRx()
{
  for (;;) {
    struct can_frame raw_frame{};
    ssize_t nbytes = recv(socket_fd_, &raw_frame, sizeof(raw_frame), 0);
    if (nbytes < 0) {
      // EAGAIN/EWOULDBLOCK: no more frames waiting right now.
      break;
    }
    if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
      continue;
    }

    CanardCANFrame frame{};
    frame.id = raw_frame.can_id;
    frame.data_len = raw_frame.can_dlc;
    frame.canfd = false;
    std::memcpy(frame.data, raw_frame.data, raw_frame.can_dlc);

    // Must be a real, monotonically increasing microsecond timestamp, not a constant: libcanard's
    // multi-frame reassembly (canard.c's canardHandleRxFrame) treats rx_state->timestamp_usec==0
    // as "state never initialized" (need_restart's `not_initialized` check). A hardcoded 0 here
    // made every second-and-later frame of any multi-frame transfer look uninitialized again,
    // wiping the already-buffered payload and aborting with RX_MISSED_START -- so esc.Status
    // (~14 bytes, 3 frames) and actuator.Status (~9 bytes, 2 frames) were silently dropped in
    // their entirety, every time, while single-frame transfers were unaffected.
    const auto now_usec = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    canardHandleRxFrame(&canard_ins_, &frame, static_cast<uint64_t>(now_usec));
  }
}

void VescDroneCanSystem::pumpTxQueue()
{
  for (const CanardCANFrame * txf = canardPeekTxQueue(&canard_ins_); txf != nullptr;
       txf = canardPeekTxQueue(&canard_ins_)) {
    struct can_frame raw_frame{};
    raw_frame.can_id = txf->id;
    raw_frame.can_dlc = txf->data_len;
    std::memcpy(raw_frame.data, txf->data, txf->data_len);

    ssize_t sent = send(socket_fd_, &raw_frame, sizeof(raw_frame), 0);
    if (sent < 0) {
      // Transient send failure (e.g. bus momentarily busy) -- drop and keep going rather than
      // stalling the whole write() cycle.
      RCLCPP_WARN_THROTTLE(logger(), clock_, 1000, "CAN send failed: %s", std::strerror(errno));
    }
    canardPopTxQueue(&canard_ins_);
  }
}

hardware_interface::return_type VescDroneCanSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  pumpRx();

  const double dt = period.seconds();

  for (auto & joint : drive_joints_) {
    // esc.Status carries no position, so the wheel angle is integrated from the reported speed.
    // Good enough for joint_state_broadcaster and rviz; diff_drive_controller runs on velocity
    // (position_feedback: false) precisely because this is dead reckoning, not a real encoder.
    joint.position_state += joint.velocity_state * dt;
    set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION, joint.position_state);
    set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, joint.velocity_state);
  }

  for (const auto & joint : steering_joints_) {
    set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION, joint.position_state);
    set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, joint.velocity_state);
  }

  for (const auto & telemetry : esc_telemetry_) {
    set_state(telemetry.name + "/voltage", telemetry.voltage);
    set_state(telemetry.name + "/current", telemetry.current);
    set_state(telemetry.name + "/temperature", telemetry.temperature);
  }

  for (const auto & sensors : steering_sensors_) {
    set_state(sensors.name + "/home_0deg", sensors.home_0deg);
    set_state(sensors.name + "/home_90deg", sensors.home_90deg);
  }

  return hardware_interface::return_type::OK;
}

void VescDroneCanSystem::broadcastRpmCommand(bool force_stop)
{
  // RPMCommand is a broadcast array indexed by esc_index: every VESC on the bus receives the same
  // message and picks out its own slot, so the array has to be long enough to reach the highest
  // index in use. Slots that belong to no configured joint stay at zero.
  uint8_t max_index = 0;
  for (const auto & joint : drive_joints_) {
    max_index = std::max(max_index, joint.esc_index);
  }

  // force_stop (set by write() when the esc-presence watchdog trips) leaves every slot at its
  // zero-initialized default -- deliberately skipping get_command() below, not just clamping its
  // result, so a missing ESC halts every OTHER drive wheel too, not only the one that dropped.
  std::vector<int32_t> rpm(static_cast<size_t>(max_index) + 1, 0);
  if (!force_stop) {
    for (const auto & joint : drive_joints_) {
      double value = wheelRadPerSecToCommandRpm(
        get_command(joint.name + "/" + hardware_interface::HW_IF_VELOCITY));
      if (!std::isfinite(value)) {
        value = 0.0;
      }
      value = std::clamp(value, -kEscRpmMax, kEscRpmMax);
      rpm[joint.esc_index] = static_cast<int32_t>(std::lround(value));
    }
  }

  uavcan_equipment_esc_RPMCommand msg{};
  msg.rpm.len = static_cast<uint8_t>(rpm.size());
  msg.rpm.data = rpm.data();

  uint8_t buf[UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_MAX_SIZE];
  uint32_t nbytes = uavcan_equipment_esc_RPMCommand_encode(&msg, buf);

  static uint8_t transfer_id = 0;
  canardBroadcast(
    &canard_ins_, UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_SIGNATURE, UAVCAN_EQUIPMENT_ESC_RPMCOMMAND_ID,
    &transfer_id, CANARD_TRANSFER_PRIORITY_HIGH, buf, static_cast<uint16_t>(nbytes), false);
}

void VescDroneCanSystem::broadcastActuatorCommand()
{
  std::vector<uavcan_equipment_actuator_Command> commands;
  commands.reserve(steering_joints_.size() * 3);
  for (auto & joint : steering_joints_) {
    uavcan_equipment_actuator_Command position_cmd{};
    position_cmd.actuator_id = joint.actuator_id;
    position_cmd.command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_POSITION;
    position_cmd.command_value =
      static_cast<float>(get_command(joint.name + "/" + hardware_interface::HW_IF_POSITION));
    commands.push_back(position_cmd);

    // Edge-triggered: only send COMMAND_TYPE_HOME the cycle this joint's "seek_home" command
    // interface value actually changes to a non-NaN value, not every cycle -- see
    // SteeringJoint::last_seek_home_command's comment for why (the firmware's own homing_tick()
    // owns the timeout once armed). Optional interface: joints that don't declare "seek_home" in
    // their URDF (e.g. a joint with no proximity sensors wired) just never send it.
    if (has_command(joint.name + "/seek_home")) {
      double seek_home = get_command(joint.name + "/seek_home");
      bool is_new_command =
        std::isfinite(seek_home) &&
        (!std::isfinite(joint.last_seek_home_command) ||
         seek_home != joint.last_seek_home_command);
      if (is_new_command) {
        uavcan_equipment_actuator_Command home_cmd{};
        home_cmd.actuator_id = joint.actuator_id;
        home_cmd.command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_HOME;
        home_cmd.command_value = static_cast<float>(seek_home);
        commands.push_back(home_cmd);
      }
      joint.last_seek_home_command = seek_home;
    }

    // Level, not edge-triggered (unlike seek_home): nonzero engages/locks the brake, 0.0 releases
    // -- see canard_driver.c's COMMAND_TYPE_BRAKE handling. Resent every cycle so a dropped frame
    // self-heals rather than leaving the brake stuck in a stale state; re-sending the same value
    // is a no-op GPIO write on the firmware side, not a state-machine trigger like HOME. Optional
    // interface, same reasoning as seek_home: a hardware component with no physical brake output
    // (gz_ros2_control's GazeboSimSystem) just never gets asked for it.
    //
    // Confirmed live over vcan0: in the brief window where this hardware component is active but
    // rp1_swerve_controller hasn't activated yet, get_command() here returns the interface's
    // uninitialized NaN default, and firmware's `command_value != 0.0f` reads that as true --
    // i.e. ENGAGE. That's a beneficial fail-safe (steering holds stiff before any controller is
    // in charge, rather than going limp), not a bug -- don't special-case NaN to avoid sending
    // it.
    if (has_command(joint.name + "/brake")) {
      uavcan_equipment_actuator_Command brake_cmd{};
      brake_cmd.actuator_id = joint.actuator_id;
      brake_cmd.command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_BRAKE;
      brake_cmd.command_value = static_cast<float>(get_command(joint.name + "/brake"));
      commands.push_back(brake_cmd);
    }
  }

  uavcan_equipment_actuator_ArrayCommand msg{};
  msg.commands.len = static_cast<uint8_t>(commands.size());
  msg.commands.data = commands.data();

  uint8_t buf[UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_MAX_SIZE];
  uint32_t nbytes = uavcan_equipment_actuator_ArrayCommand_encode(&msg, buf);

  static uint8_t transfer_id = 0;
  canardBroadcast(
    &canard_ins_, UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE,
    UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID, &transfer_id, CANARD_TRANSFER_PRIORITY_MEDIUM, buf,
    static_cast<uint16_t>(nbytes), false);
}

hardware_interface::return_type VescDroneCanSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!drive_joints_.empty()) {
    std::vector<uint8_t> missing;
    const bool all_present = allDriveEscsPresent(clock_.now(), &missing);
    if (!all_present) {
      std::ostringstream missing_str;
      for (size_t i = 0; i < missing.size(); ++i) {
        if (i > 0) {
          missing_str << ", ";
        }
        missing_str << static_cast<int>(missing[i]);
      }
      RCLCPP_ERROR_THROTTLE(
        logger(), clock_, 1000,
        "vesc_dronecan_driver: esc_index [%s] missing from the CAN bus (no esc.Status within "
        "%.2fs) -- forcing zero RPMCommand to ALL %zu drive wheel(s)",
        missing_str.str().c_str(), esc_timeout_sec_, drive_joints_.size());
    }
    broadcastRpmCommand(!all_present);
  }
  if (!steering_joints_.empty()) {
    broadcastActuatorCommand();
  }

  pumpTxQueue();

  return hardware_interface::return_type::OK;
}

bool VescDroneCanSystem::allDriveEscsPresent(
  const rclcpp::Time & now, std::vector<uint8_t> * missing) const
{
  bool all_present = true;
  for (const auto & joint : drive_joints_) {
    if ((now - joint.last_status_time).seconds() > esc_timeout_sec_) {
      all_present = false;
      if (missing != nullptr) {
        missing->push_back(joint.esc_index);
      }
    }
  }
  return all_present;
}

void VescDroneCanSystem::handleEscStatus(CanardRxTransfer * transfer)
{
  uavcan_equipment_esc_Status status{};
  if (uavcan_equipment_esc_Status_decode(transfer, transfer->payload_len, &status, nullptr) < 0) {
    return;
  }

  for (auto & joint : drive_joints_) {
    if (joint.esc_index == status.esc_index) {
      joint.velocity_state = statusRpmToWheelRadPerSec(static_cast<double>(status.rpm));
      joint.last_status_time = clock_.now();
      break;
    }
  }

  for (auto & telemetry : esc_telemetry_) {
    if (telemetry.esc_index == status.esc_index) {
      telemetry.voltage = status.voltage;
      telemetry.current = status.current;
      telemetry.temperature = status.temperature - kKelvinOffset;
      break;
    }
  }
}

void VescDroneCanSystem::handleActuatorStatus(CanardRxTransfer * transfer)
{
  uavcan_equipment_actuator_Status status{};
  if (uavcan_equipment_actuator_Status_decode(transfer, transfer->payload_len, &status, nullptr) < 0) {
    return;
  }

  for (auto & joint : steering_joints_) {
    if (joint.actuator_id == status.actuator_id) {
      joint.position_state = status.position;
      joint.velocity_state = status.speed;
      break;
    }
  }

  for (auto & sensors : steering_sensors_) {
    if (sensors.actuator_id == status.actuator_id) {
      sensors.home_0deg = status.home_0deg ? 1.0 : 0.0;
      sensors.home_90deg = status.home_90deg ? 1.0 : 0.0;
      break;
    }
  }
}

void VescDroneCanSystem::onTransferReceived(CanardInstance * ins, CanardRxTransfer * transfer)
{
  auto * self = static_cast<VescDroneCanSystem *>(canardGetUserReference(ins));
  if (self == nullptr) {
    return;
  }

  switch (transfer->data_type_id) {
    case UAVCAN_EQUIPMENT_ESC_STATUS_ID:
      self->handleEscStatus(transfer);
      break;
    case UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID:
      self->handleActuatorStatus(transfer);
      break;
    default:
      break;
  }
}

bool VescDroneCanSystem::shouldAcceptTransfer(
  const CanardInstance * /*ins*/, uint64_t * out_data_type_signature, uint16_t data_type_id,
  CanardTransferType /*transfer_type*/, uint8_t /*source_node_id*/)
{
  switch (data_type_id) {
    case UAVCAN_EQUIPMENT_ESC_STATUS_ID:
      *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE;
      return true;
    case UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID:
      *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE;
      return true;
    default:
      return false;
  }
}

}  // namespace vesc_dronecan_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(vesc_dronecan_driver::VescDroneCanSystem, hardware_interface::SystemInterface)
