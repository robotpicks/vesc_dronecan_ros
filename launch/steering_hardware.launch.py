"""Brings up the rp1_hardware_interface SteeringHardware plugin under controller_manager, plus
joint_state_broadcaster and a position_controllers/JointGroupPositionController claiming the
two steering joints' position command interfaces -- the real closed-loop-wiring test (still no
encoder on the bench VESCs, so position feedback itself isn't meaningful yet, but this confirms
a real ros2_control controller can claim and drive the DroneCAN-backed command interface).

Controllers are spawned strictly sequentially (TimerAction delay, then chained via
OnProcessExit) rather than concurrently -- controller_manager's service handling runs on the
same thread as its real-time update loop (see ros-controls/ros2_control#2808), and two spawner
processes hitting its services at once was observed to hang controller_manager entirely in this
sandboxed environment (no CAP_SYS_NICE for real RT scheduling). One spawner at a time avoids
that contention.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rp1_hardware_interface')
    urdf_path = os.path.join(pkg_share, 'urdf', 'rp1_steering.urdf')
    controller_manager_config = os.path.join(pkg_share, 'config', 'controller_manager.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[{'robot_description': robot_description}, controller_manager_config],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster'],
    )

    steering_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='steering_position_controller_spawner',
        output='screen',
        arguments=['steering_position_controller'],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        # Give controller_manager a few seconds to finish activating the hardware before the
        # first spawner hits its services.
        TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[steering_position_controller_spawner],
            )
        ),
    ])
