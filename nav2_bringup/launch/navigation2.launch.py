
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    localization_parent_frame = LaunchConfiguration('localization_parent_frame')
    localization_child_frame = LaunchConfiguration('localization_child_frame')
    localization_x = LaunchConfiguration('localization_x')
    localization_y = LaunchConfiguration('localization_y')
    localization_z = LaunchConfiguration('localization_z')
    localization_yaw = LaunchConfiguration('localization_yaw')
    localization_tf_time_offset = LaunchConfiguration('localization_tf_time_offset')
    bringup_dir = get_package_share_directory('nav2_bringup')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            bringup_dir,
            'models',
            'myworld2',
            'myworld2.yaml'))

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            bringup_dir,
            'params',
            param_file_name))

    nav2_launch_file_dir = os.path.join(bringup_dir, 'launch')

    rviz_config_dir = os.path.join(
        bringup_dir,
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'localization_parent_frame',
            default_value='map',
            description='Parent frame published by locationpub'),

        DeclareLaunchArgument(
            'localization_child_frame',
            default_value='turtlebot3_waffle',
            description='Child frame published by locationpub'),

        DeclareLaunchArgument(
            'localization_x',
            default_value='0.569',
            description='locationpub x translation'),

        DeclareLaunchArgument(
            'localization_y',
            default_value='0.541',
            description='locationpub y translation'),

        DeclareLaunchArgument(
            'localization_z',
            default_value='0.0',
            description='locationpub z translation'),

        DeclareLaunchArgument(
            'localization_yaw',
            default_value='0.0',
            description='locationpub yaw rotation'),

        DeclareLaunchArgument(
            'localization_tf_time_offset',
            default_value='0.2',
            description='Seconds subtracted from locationpub TF stamp'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/localization_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'localization_parent_frame': localization_parent_frame,
                'localization_child_frame': localization_child_frame,
                'localization_x': localization_x,
                'localization_y': localization_y,
                'localization_z': localization_z,
                'localization_yaw': localization_yaw,
                'localization_tf_time_offset': localization_tf_time_offset}.items(),
        ),

        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': param_dir}.items(),
                ),
            ],
        ),

        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_dir],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen'),
            ],
        ),
    ])
