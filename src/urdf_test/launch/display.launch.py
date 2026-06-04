from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, AndSubstitution, OrSubstitution, NotSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = Command(['xacro ', PathJoinSubstitution([FindPackageShare('urdf_test'), 'description', 'model.urdf.xacro'])])
    params = {'robot_description': urdf, 'use_sim_time': use_sim_time}

    gz_launch_path = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])

    rz_bridge_cfg_path = PathJoinSubstitution([FindPackageShare('urdf_test'), 'config', 'ros_gz_example_bridge.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
       DeclareLaunchArgument(
            'joint_gui',
            default_value='True',
            description='Flag to start joint_state_publisher_gui'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            condition=IfCondition(LaunchConfiguration("use_sim_time")),
            launch_arguments={
                # 'gz_args': PathJoinSubstitution([example_pkg_path, 'worlds/example_world.sdf']),  # Replace with your own world file
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_spawn_model.launch.py'])]),
            condition=IfCondition(LaunchConfiguration("use_sim_time")),
            launch_arguments=[
                        # ('world', world),
                        #   ('file', file),
                        #   ('model_string', model_string),
                          ('topic', 'robot_description'),
                          ('entity_name', 'robot'),
                        #   ('allow_renaming', allow_renaming),
                        #   ('x', x),
                        #   ('y', y),
                        #   ('z', z),
                        #   ('R', roll),
                        #   ('P', pitch),
                        #   ('Y', yaw), ]),
            ]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                ['config_file:=', rz_bridge_cfg_path]],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
        ) ,
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[params],
            condition=IfCondition(
                AndSubstitution(
                    NotSubstitution(LaunchConfiguration('joint_gui')),
                    NotSubstitution(LaunchConfiguration("use_sim_time"))
                )),
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(
                AndSubstitution(
                    LaunchConfiguration('joint_gui'),
                    NotSubstitution(LaunchConfiguration("use_sim_time"))
                )),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('urdf_test'), 'robot.rviz',])],
        ),
    ])