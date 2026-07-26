#include "rp1_hardware_interface/rp1_hardware.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>

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

namespace rp1_hardware_interface
{

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("rp1_hardware_interface"); }

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

hardware_interface::CallbackReturn Rp1Hardware::on_init(
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

  if (gear_ratio_ <= 0.0 || motor_pole_pairs_ <= 0.0) {
    RCLCPP_ERROR(
      logger(), "gear_ratio (%f) and motor_pole_pairs (%f) must both be positive", gear_ratio_,
      motor_pole_pairs_);
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

  // <gpio> blocks carry the per-ESC telemetry that has no joint interface of its own.
  esc_telemetry_.clear();
  for (const auto & gpio : info.gpios) {
    auto it = gpio.parameters.find("esc_index");
    if (it == gpio.parameters.end()) {
      RCLCPP_ERROR(
        logger(), "GPIO '%s' is missing the required 'esc_index' parameter", gpio.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    EscTelemetry telemetry;
    telemetry.name = gpio.name;
    telemetry.esc_index = static_cast<uint8_t>(std::stoi(it->second));
    esc_telemetry_.push_back(telemetry);
  }

  canard_memory_pool_.resize(4096);

  RCLCPP_INFO(
    logger(), "rp1_hardware_interface configured: %zu drive, %zu steering, %zu ESC telemetry",
    drive_joints_.size(), steering_joints_.size(), esc_telemetry_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rp1Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
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
    "rp1_hardware_interface up on %s (node_id=%d, %zu drive + %zu steering joint(s), "
    "gear_ratio=%.4f, pole_pairs=%.1f, command_rpm_is_erpm=%s)",
    can_iface_.c_str(), local_node_id_, drive_joints_.size(), steering_joints_.size(), gear_ratio_,
    motor_pole_pairs_, command_rpm_is_erpm_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rp1Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

double Rp1Hardware::wheelRadPerSecToCommandRpm(double rad_per_sec) const
{
  // wheel rad/s -> wheel RPM -> motor RPM, then (because the firmware feeds RPMCommand straight
  // into mc_interface_set_pid_speed without a pole-pair conversion) motor RPM -> ERPM.
  double rpm = rad_per_sec * kRadPerSecToRpm * gear_ratio_;
  if (command_rpm_is_erpm_) {
    rpm *= motor_pole_pairs_;
  }
  return rpm;
}

double Rp1Hardware::statusRpmToWheelRadPerSec(double status_rpm) const
{
  // esc.Status.rpm is already mechanical motor RPM -- sendEscStatus() divides ERPM by pole pairs
  // before transmitting -- so only the gearing has to come back out here.
  return (status_rpm / gear_ratio_) * kRpmToRadPerSec;
}

void Rp1Hardware::pumpRx()
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

    canardHandleRxFrame(&canard_ins_, &frame, 0);
  }
}

void Rp1Hardware::pumpTxQueue()
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

hardware_interface::return_type Rp1Hardware::read(
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

  return hardware_interface::return_type::OK;
}

void Rp1Hardware::broadcastRpmCommand()
{
  // RPMCommand is a broadcast array indexed by esc_index: every VESC on the bus receives the same
  // message and picks out its own slot, so the array has to be long enough to reach the highest
  // index in use. Slots that belong to no configured joint stay at zero.
  uint8_t max_index = 0;
  for (const auto & joint : drive_joints_) {
    max_index = std::max(max_index, joint.esc_index);
  }

  std::vector<int32_t> rpm(static_cast<size_t>(max_index) + 1, 0);
  for (const auto & joint : drive_joints_) {
    double value = wheelRadPerSecToCommandRpm(
      get_command(joint.name + "/" + hardware_interface::HW_IF_VELOCITY));
    if (!std::isfinite(value)) {
      value = 0.0;
    }
    value = std::clamp(value, -kEscRpmMax, kEscRpmMax);
    rpm[joint.esc_index] = static_cast<int32_t>(std::lround(value));
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

void Rp1Hardware::broadcastActuatorCommand()
{
  std::vector<uavcan_equipment_actuator_Command> commands;
  commands.reserve(steering_joints_.size());
  for (const auto & joint : steering_joints_) {
    uavcan_equipment_actuator_Command cmd{};
    cmd.actuator_id = joint.actuator_id;
    cmd.command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_POSITION;
    cmd.command_value =
      static_cast<float>(get_command(joint.name + "/" + hardware_interface::HW_IF_POSITION));
    commands.push_back(cmd);
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

hardware_interface::return_type Rp1Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!drive_joints_.empty()) {
    broadcastRpmCommand();
  }
  if (!steering_joints_.empty()) {
    broadcastActuatorCommand();
  }

  pumpTxQueue();

  return hardware_interface::return_type::OK;
}

void Rp1Hardware::handleEscStatus(CanardRxTransfer * transfer)
{
  uavcan_equipment_esc_Status status{};
  if (uavcan_equipment_esc_Status_decode(transfer, transfer->payload_len, &status, nullptr) < 0) {
    return;
  }

  for (auto & joint : drive_joints_) {
    if (joint.esc_index == status.esc_index) {
      joint.velocity_state = statusRpmToWheelRadPerSec(static_cast<double>(status.rpm));
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

void Rp1Hardware::handleActuatorStatus(CanardRxTransfer * transfer)
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
}

void Rp1Hardware::onTransferReceived(CanardInstance * ins, CanardRxTransfer * transfer)
{
  auto * self = static_cast<Rp1Hardware *>(canardGetUserReference(ins));
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

bool Rp1Hardware::shouldAcceptTransfer(
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

}  // namespace rp1_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rp1_hardware_interface::Rp1Hardware, hardware_interface::SystemInterface)
