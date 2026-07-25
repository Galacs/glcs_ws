from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('cl57pr_hardware_interface')
    urdf_path = os.path.join(pkg_share, 'urdf', 'cl57pr_test.urdf')
    controllers_yaml = os.path.join(pkg_share, 'bringup', 'config', 'controllers.yaml')

    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content}, controllers_yaml],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
        output='screen',
    )

    # Spawn the position controller only after the joint_state_broadcaster is up.
    delay_position_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_position_controller_after_jsb,
    ])
