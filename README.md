# vesc_dronecan_ros

A **ros2_control hardware component for VESC motor controllers over DroneCAN**. It presents your
VESCs to ros2_control as ordinary joints, so stock controllers — `diff_drive_controller`,
`joint_trajectory_controller`, anything else — drive them with no robot-specific glue:

- **driven joints** — velocity command via `uavcan.equipment.esc.RPMCommand`, feedback from
  `esc.Status`
- **steered joints** — position command via `uavcan.equipment.actuator.ArrayCommand`, feedback
  from `actuator.Status`
- **per-ESC pack telemetry** — voltage / current / temperature exported as `<gpio>` state
  interfaces

It talks SocketCAN directly using a vendored copy of the same **libcanard** codec the VESC
firmware itself uses, so the encoder and the decoder are the same code on both ends of the wire.

## Why this exists

VESC firmware has a built-in UAVCAN/DroneCAN mode, so a CAN adapter can wire straight to a bus of
VESCs with **no bridge MCU and no translator node**. But nothing in ros2_control speaks DroneCAN,
and the usual alternatives don't fit:

- `vesc_ros`-style drivers speak VESC's **native serial/CAN protocol**, one UART per controller —
  which doesn't scale to a shared bus and doesn't use the firmware's own UAVCAN mode.
- A Python DroneCAN bridge node works, but puts a topic hop and a GIL between the controller and
  the wire, and forces you to hand-write the kinematics that stock controllers already implement.

Being a `SystemInterface` instead means the command path is `controller → hardware → CAN` inside
one real-time loop, and you get `diff_drive_controller`'s kinematics, odometry and `cmd_vel`
watchdog for free.

## Install

```bash
cd ~/ros2_ws/src
git clone https://github.com/robotpicks/vesc_dronecan_ros.git
cd ~/ros2_ws
rosdep install --from-paths src/vesc_dronecan_ros --ignore-src -r -y
colcon build --packages-select vesc_dronecan_driver
source install/setup.bash
```

Needs `ros2_control` (`hardware_interface`, `pluginlib`); no Python CAN libraries and no
`dronecan` package — the codec is vendored C.

## Usage

Add a `<ros2_control>` block to your robot description. A complete, commented example is in
[`vesc_dronecan_driver/urdf/example_4wheel.urdf`](vesc_dronecan_driver/urdf/example_4wheel.urdf):

```xml
<ros2_control name="my_robot" type="system">
  <hardware>
    <plugin>vesc_dronecan_driver/VescDroneCanSystem</plugin>
    <param name="can_iface">can0</param>
    <param name="node_id">42</param>
    <param name="gear_ratio">1.0</param>
    <param name="motor_pole_pairs">7.0</param>
    <param name="command_rpm_is_erpm">true</param>
  </hardware>

  <joint name="drive_front_left">
    <param name="esc_index">0</param>
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
    <state_interface name="position"/>
  </joint>
</ros2_control>
```

### Hardware parameters

| Param | Default | Meaning |
|-------|---------|---------|
| `can_iface` | `can0` | SocketCAN interface to bind. `vcan0` for a virtual bus. |
| `node_id` | `42` | This component's DroneCAN node ID. Must not collide with any VESC's CAN ID. |
| `gear_ratio` | `1.0` | Motor revolutions per joint revolution. `1.0` = direct drive. |
| `motor_pole_pairs` | `1.0` | `si_motor_poles / 2` from VESC motor detection. |
| `command_rpm_is_erpm` | `true` | Compensate for the firmware's RPM unit asymmetry — see below. |

`can_iface` **cannot be overridden as a node parameter** — ros2_control reads hardware parameters
from the robot description and nowhere else. To make it switchable, rewrite the string in your
launch file before publishing the description.

### Joint parameters

A joint declares **exactly one** of:

| Param | Kind | Command | Feedback |
|-------|------|---------|----------|
| `esc_index` | driven wheel | `velocity` → `esc.RPMCommand` | `esc.Status` |
| `actuator_id` | steered joint | `position` → `actuator.ArrayCommand` | `actuator.Status` |

Declaring both, or neither, is a hard error at `on_init()` rather than a silent misconfiguration.

Both values are that VESC's **`uavcan_esc_index`** (App Settings → General → UAVCAN ESC index in
VESC Tool). VESC has no separate actuator-id field, so driven and steered joints share one id
space — allocate distinct values across the whole bus.

`esc.Status` carries no position, so a driven joint's `position` state is **integrated from
reported speed**. It is dead reckoning, not an encoder: configure odometry to run on velocity
(for `diff_drive_controller`, `position_feedback: false`).

## VESC-side configuration

Per VESC, over VESC Tool's USB link (a bench activity — the runtime path never uses VESC Tool):

- **CAN Mode = `VESC+UAVCAN`**, not plain `UAVCAN`. Plain UAVCAN takes exclusive use of the bus
  and stops VESC Tool and normal VESC CAN diagnostics from working on the shared bus.
- **CAN ID** — doubles as the DroneCAN node ID in `VESC+UAVCAN` mode; unique on the bus.
- **UAVCAN ESC index** — the `esc_index` / `actuator_id` your URDF declares.

### RPM units — the thing that will bite you

VESC firmware is asymmetric about RPM on the wire:

```
sendEscStatus():          status.rpm = get_rpm() / pole_pairs   -> MECHANICAL rpm
handle_esc_rpm_command(): set_pid_speed(rpm_val)  (no scaling)  -> ERPM
```

So a value read back from `Status` **cannot be commanded verbatim** — it is off by the pole-pair
count. `command_rpm_is_erpm: true` (the default) compensates on the command side, and matches
firmware as it stands. Set it `false` only if your firmware scales the command side too.

If your robot drives at `pole_pairs` times the commanded speed, this parameter is why.

## Firmware support for steered joints

Driven joints work against **stock VESC firmware** — `esc.RPMCommand` is handled upstream.

Steered joints do **not**. Upstream VESC firmware has no `uavcan.equipment.actuator` support at
all — no `ArrayCommand` handler, no `Status` publisher. You need firmware that adds it, such as
[robotpicks/bldc](https://github.com/robotpicks/bldc), where `handle_actuator_array_command()`
maps `COMMAND_TYPE_POSITION` onto `mc_interface_set_pid_pos()`.

Position control also needs a **real encoder** on the steered VESC — `mc_interface_set_pid_pos`
requires FOC position feedback. Without one the reported position is FOC reacting to phantom
feedback, not a joint angle. Prefer an **absolute** encoder so the joint knows its angle at
power-on with no homing.

## Getting the ESC telemetry to actually appear

The `<gpio>` telemetry reaches `/dynamic_joint_states` only if `joint_state_broadcaster` is
configured with **both**:

```yaml
joint_state_broadcaster:
  ros__parameters:
    publish_dynamic_joint_states: true   # default false: topic not advertised at all
    use_urdf_to_filter: false            # default true: drops non-joint prefixes, i.e. all gpios
```

Both defaults suppress it, and neither failure is reported anywhere —
`ros2 control list_hardware_interfaces` lists all the interfaces either way, while the topic stays
empty or absent.

## Testing without hardware

**No CAN at all** — swap the plugin for ros2_control's mock:

```xml
<plugin>mock_components/GenericSystem</plugin>
```

Commands loop straight back to states. Note it leaves unclaimed `<gpio>` states at NaN, so it
does not exercise the telemetry path.

**Real DroneCAN framing, virtual bus** — create a `vcan0` and run a DroneCAN-speaking stand-in for
the VESCs against it:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

then point `can_iface` at `vcan0`. This exercises real encode/decode and the SocketCAN path.

## License

Apache-2.0 — see [LICENSE](LICENSE).
