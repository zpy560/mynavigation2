# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Nav2 bringup for TurtleBot3 in Gazebo Sim."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    world = LaunchConfiguration('world')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map yaml file to load')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Whether to use composed bringup')
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RViz')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'tb3_gz_view.rviz'),
        description='Full path to the RVIZ config file to use')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'tb3_gz_sim.sdf'),
        description='Full path to the Gazebo Sim world file to load')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    gz_ip_envvar = SetEnvironmentVariable('GZ_IP', '127.0.0.1')
    ign_ip_envvar = SetEnvironmentVariable('IGN_IP', '127.0.0.1')

    robot_description_file = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(robot_description_file, 'r') as infp:
        robot_description = infp.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-s -r ', world]}.items())

    bridge = Node(
        package='ros_gz_bridge',
        executable='bridge_node',
        name='nav2_gz_bridge',
        namespace=namespace,
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[{'config_file': os.path.join(bringup_dir, 'config', 'tb3_gz_bridge.yaml')}],
        arguments=['--ros-args', '--log-level', 'info'])

    static_model_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='model_to_base_broadcaster',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                   'turtlebot3_waffle', 'base_footprint'])

    static_base_to_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_scan_broadcaster',
        output='screen',
        arguments=['0.0', '0.0', '0.028', '0.0', '0.0', '0.0',
                   'base_scan', 'turtlebot3_waffle/base_scan/hls_lfcd_lds'])

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': 'False',
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(gz_ip_envvar)
    ld.add_action(ign_ip_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(bridge)
    ld.add_action(static_model_to_base)
    ld.add_action(static_base_to_scan)
    ld.add_action(bringup)
    ld.add_action(rviz)

    return ld
