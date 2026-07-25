#include "rp1_hardware_interface/steering_hardware.hpp"

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
}

namespace rp1_hardware_interface
{

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("rp1_hardware_interface"); }
}  // namespace

hardware_interface::CallbackReturn SteeringHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // Base on_init() parses the URDF-declared state/command interfaces (position/velocity per
  // joint). Current ros2_control passes a HardwareComponentInterfaceParams (HardwareInfo plus a
  // weak_ptr to the controller manager's executor); the old HardwareInfo-only overload is gone,
  // so the URDF data is reached through params.hardware_info.
  auto base_result = hardware_interface::SystemInterface::on_init(params);
  if (base_result != hardware_interface::CallbackReturn::SUCCESS) {
    return base_result;
  }

  const auto & info = params.hardware_info;

  auto iface_it = info.hardware_parameters.find("can_iface");
  if (iface_it != info.hardware_parameters.end()) {
    can_iface_ = iface_it->second;
  }

  auto node_id_it = info.hardware_parameters.find("node_id");
  if (node_id_it != info.hardware_parameters.end()) {
    local_node_id_ = static_cast<uint8_t>(std::stoi(node_id_it->second));
  }

  joints_.clear();
  joints_.reserve(info.joints.size());
  for (const auto & joint : info.joints) {
    auto actuator_id_it = joint.parameters.find("actuator_id");
    if (actuator_id_it == joint.parameters.end()) {
      RCLCPP_ERROR(
        logger(), "Joint '%s' is missing the required 'actuator_id' parameter", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    SteeringJoint joint_data;
    joint_data.name = joint.name;
    joint_data.actuator_id = static_cast<uint8_t>(std::stoi(actuator_id_it->second));
    joints_.push_back(joint_data);
  }

  canard_memory_pool_.resize(4096);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SteeringHardware::on_activate(
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
    logger(), "rp1_hardware_interface up on %s (node_id=%d, %zu joint(s))", can_iface_.c_str(),
    local_node_id_, joints_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SteeringHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

void SteeringHardware::pumpRx()
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

void SteeringHardware::pumpTxQueue()
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
      // stalling the whole write() cycle; mirrors rp1_dronecan_bridge's broadcast error handling.
      RCLCPP_WARN_THROTTLE(
        logger(), clock_, 1000, "CAN send failed: %s", std::strerror(errno));
    }
    canardPopTxQueue(&canard_ins_);
  }
}

hardware_interface::return_type SteeringHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  pumpRx();

  for (const auto & joint : joints_) {
    set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION, joint.position_state);
    set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, joint.velocity_state);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SteeringHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (joints_.empty()) {
    return hardware_interface::return_type::OK;
  }

  std::vector<uavcan_equipment_actuator_Command> commands;
  commands.reserve(joints_.size());
  for (const auto & joint : joints_) {
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

  pumpTxQueue();

  return hardware_interface::return_type::OK;
}

void SteeringHardware::handleActuatorStatus(CanardRxTransfer * transfer)
{
  uavcan_equipment_actuator_Status status{};
  if (uavcan_equipment_actuator_Status_decode(transfer, transfer->payload_len, &status, nullptr) < 0) {
    return;
  }

  for (auto & joint : joints_) {
    if (joint.actuator_id == status.actuator_id) {
      joint.position_state = status.position;
      joint.velocity_state = status.speed;
      break;
    }
  }
}

void SteeringHardware::onTransferReceived(CanardInstance * ins, CanardRxTransfer * transfer)
{
  auto * self = static_cast<SteeringHardware *>(canardGetUserReference(ins));
  if (self == nullptr) {
    return;
  }

  if (transfer->data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID) {
    self->handleActuatorStatus(transfer);
  }
}

bool SteeringHardware::shouldAcceptTransfer(
  const CanardInstance * /*ins*/, uint64_t * out_data_type_signature, uint16_t data_type_id,
  CanardTransferType /*transfer_type*/, uint8_t /*source_node_id*/)
{
  if (data_type_id == UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID) {
    *out_data_type_signature = UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE;
    return true;
  }
  return false;
}

}  // namespace rp1_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rp1_hardware_interface::SteeringHardware, hardware_interface::SystemInterface)
