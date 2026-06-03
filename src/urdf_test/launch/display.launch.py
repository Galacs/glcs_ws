from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # pkg_path = launch_ros.substitutions.FindPackageShare()
    urdf = Command(['xacro ', PathJoinSubstitution([FindPackageShare('urdf_test'), 'urdf', 'model.urdf.xacro'])])

    return LaunchDescription([
       DeclareLaunchArgument(
            'joint_gui',
            default_value='True',
            description='Flag to start joint_state_publisher_gui'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}],
        ) ,
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'robot_description': urdf}],
            condition=UnlessCondition(LaunchConfiguration('joint_gui')),
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('joint_gui')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('urdf_test'), 'robot.rviz',])],
        ),
    ])