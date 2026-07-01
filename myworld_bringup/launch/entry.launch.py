import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='myworld2')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    bringup_dir = get_package_share_directory('myworld_bringup')
    launch_file_dir = os.path.dirname(__file__)
    myworld_dir = os.path.join(bringup_dir, 'models', 'myworld2')
    map_file = os.path.join(myworld_dir, 'myworld2.yaml')
    params_file = os.path.join(bringup_dir, 'params', 'myworld2.yaml')
    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')
    robot_description_file = os.path.join(myworld_dir, 'diffbot.sdf')
    robot_description = xacro.parse(open(robot_description_file))
    xacro.process_doc(robot_description)

    resource_path = [
            os.path.join('/opt/ros/humble', 'share'),
            ':' + os.path.join(bringup_dir, 'models'),
            ':' + myworld_dir,
            ':' + os.path.join(myworld_dir, 'models')]

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path)

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path)

    ign_partition = SetEnvironmentVariable(
        name='IGN_PARTITION',
        value='myworld_bringup')

    gz_partition = SetEnvironmentVariable(
        name='GZ_PARTITION',
        value='myworld_bringup')

    # Spawn robot
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'diffbot',
                   '-file', PathJoinSubstitution([
                        bringup_dir,
                        "models", "myworld2", "diffbot.sdf"]),
                   '-allow_renaming', 'false',
                   '-x', '-3.071',
                   '-y', '3.583',
                   '-z', '0.01'],
        )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description.toxml()}])

    world_only = os.path.join(myworld_dir, 'world_only.sdf')

    return LaunchDescription([
        ign_resource_path,
        gz_resource_path,
        ign_partition,
        gz_partition,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('ign_args', [' -r -v 3 ' +
                             world_only
                             ])]),

        ignition_spawn_entity,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),

        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to start RViz'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/ros_ign_bridge.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        robot_state_publisher,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/localization_launch.py']),
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file}.items(),
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
