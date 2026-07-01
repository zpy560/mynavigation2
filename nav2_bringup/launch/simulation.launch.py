import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='turtlebot3_world')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    launch_file_dir = os.path.join(nav2_bringup_dir, 'launch')
    map_file = os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml')
    params_file = os.path.join(nav2_bringup_dir, 'params', 'waffle.yaml')

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', value=[
            os.path.join('/opt/ros/humble', 'share'),
            ':' +
            os.path.join(nav2_bringup_dir, 'models')])

    # Spawn robot
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-entity', TURTLEBOT3_MODEL,
                   '-name', TURTLEBOT3_MODEL,
                   '-file', PathJoinSubstitution([
                        nav2_bringup_dir,
                        "models", "turtlebot3", "model.sdf"]),
                   '-allow_renaming', 'true',
                   '-x', '-2.0',
                   '-y', '-0.5',
                   '-z', '0.01'],
        )

    # Spawn world
    ignition_spawn_world = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', PathJoinSubstitution([
                        nav2_bringup_dir,
                        "models", "worlds", "model.sdf"]),
                   '-allow_renaming', 'false'],
        )

    world_only = os.path.join(nav2_bringup_dir, 'models', 'worlds', 'world_only.sdf')

    return LaunchDescription([
        ign_resource_path,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v 3 ' +
                              world_only
                             ])]),

        TimerAction(period=3.0, actions=[ignition_spawn_world, ignition_spawn_entity]),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/ros_ign_bridge.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([launch_file_dir, '/navigation2.launch.py']),
                    launch_arguments={
                        'map': map_file,
                        'use_sim_time': use_sim_time,
                        'params_file': params_file,
                        'localization_child_frame': 'odom',
                        'localization_x': '0.0',
                        'localization_y': '0.0',
                        'localization_z': '0.0',
                        'localization_yaw': '0.0',
                        'localization_tf_time_offset': '0.0'}.items(),
                ),
            ],
        ),
    ])
