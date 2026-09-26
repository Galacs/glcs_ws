from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('rviz')

    nav2_bringup_share = FindPackageShare('nav2_bringup')
    default_params_file = PathJoinSubstitution(
        [FindPackageShare('bnav'), 'config', 'nav2_params.yaml'])
    default_map_file = PathJoinSubstitution(
        [FindPackageShare('bnav'), 'maps', 'map.yaml'])
    default_bt_xml = PathJoinSubstitution(
    [FindPackageShare('bnav'), 'behavior_trees', 'alternate_goals.xml'])

    navigation_launch_path = PathJoinSubstitution(
        [nav2_bringup_share, 'launch', 'navigation_launch.py'])
    rviz_launch_path = PathJoinSubstitution(
        [nav2_bringup_share, 'launch', 'rviz_launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params_file,
            description='Full path to the nav2 params file'),
        DeclareLaunchArgument(
            'map', default_value=default_map_file,
            description='Full path to the static map yaml file'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically bring nodes to the active state'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch the standard Nav2 RViz view alongside navigation'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml_file,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server'],
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': autostart,
                # 'default_bt_xml_filename': default_bt_xml,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch_path),
            condition=IfCondition(use_rviz),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        Node(
            package='bnav',
            executable='alternate_goals_mission',
            name='alternate_goals_mission',
            output='screen',
        ),

    ])