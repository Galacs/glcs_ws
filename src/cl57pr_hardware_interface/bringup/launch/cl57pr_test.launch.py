import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('cl57pr_hardware_interface')
    urdf_path = os.path.join(pkg_share, 'urdf', 'cl57pr_test.urdf')
    controllers_yaml_path = os.path.join(pkg_share, 'bringup', 'config', 'controllers.yaml')

    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    # Controller manager parameters (types and update rate)
    controller_manager_params = {
        'update_rate': 50,
        'joint_state_broadcaster': {
            'type': 'joint_state_broadcaster/JointStateBroadcaster'
        },
        'position_controller': {
            'type': 'forward_command_controller/ForwardCommandController'
        }
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_content},
            controller_manager_params
        ],
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
        arguments=[
            'position_controller',
            '-p', controllers_yaml_path   # Use the YAML file for controller-specific parameters
        ],
        output='screen',
    )

    delay_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_position_controller,
    ])