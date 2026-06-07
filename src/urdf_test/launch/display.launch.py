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
    urdf_cam = Command(['xacro ', PathJoinSubstitution([FindPackageShare('urdf_test'), 'description', 'mcc_camera.urdf.xacro'])])
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
                'gz_args': 'empty.sdf',
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
                          ('topic', 'robot_description'),
                          ('entity_name', 'robot'),
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                   'launch',
                                   'gz_spawn_model.launch.py'])]),
            condition=IfCondition(LaunchConfiguration("use_sim_time")),
            launch_arguments=[('topic', 'mcc/robot_description'),
                              ('entity_name', 'mcc_camera'),
                              ('x', '0.0'), ('y', '0.0'), ('z', '2.0'),
                              ('R', '0.0'), ('P', '1.57'), ('Y', '0.0')],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                ['config_file:=', rz_bridge_cfg_path]],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration("use_sim_time")),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher_mcc',
            output='screen',
            namespace='mcc',
            parameters=[{'robot_description': urdf_cam, 'use_sim_time': use_sim_time}],
        ),
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
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '2',
                '--yaw', '0', '--pitch', '1.57', '--roll', '0',
                '--frame-id', 'map', '--child-frame-id', 'camera_link']
        ),
        # Node(
        #     package='ros_gz_image',
        #     executable='image_bridge',
        #     arguments=['/mcc/camera/image_raw'],
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #         'override_frame_id': 'camera_link_optical',
        #     }],
        #     condition=IfCondition(LaunchConfiguration("use_sim_time")),
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('urdf_test'), 'robot.rviz',])],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])