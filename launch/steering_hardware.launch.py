"""Brings up just the rp1_hardware_interface SteeringHardware plugin under controller_manager,
with no controllers loaded yet -- lets ros2_control claim/activate the hardware and drive its
read()/write() cycle (real DroneCAN traffic on can0) for standalone testing, ahead of
ros2_controllers (position_controllers/joint_state_broadcaster) being available.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rp1_hardware_interface')
    urdf_path = os.path.join(pkg_share, 'urdf', 'rp1_steering.urdf')
    controller_manager_config = os.path.join(pkg_share, 'config', 'controller_manager.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[{'robot_description': robot_description}, controller_manager_config],
        ),
    ])
