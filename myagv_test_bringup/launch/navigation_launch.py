# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    # 中文说明：此文件用于启动 Nav2 的各个组件（controller、planner、smoother、lifecycle manager 等）。
    # - 支持两种启动方式：普通（每个节点单独进程）和组合（composed，多个组件加载到同一个容器进程）。
    # - 通过 LaunchArguments 可配置命名空间、是否使用仿真时间、参数文件、是否使用 composition 等。
    # - `configured_params` 会根据 namespace 与 use_sim_time/autostart 对原始 params 文件做临时替换。
    # 主要部分说明：
    #   namespace: 节点顶层命名空间，便于在多机器人场景隔离话题/参数
    #   use_sim_time: 是否使用仿真时钟（Gazebo）
    #   autostart: 是否自动激活生命周期节点
    #   use_composition: 如果为 True 则使用 composable nodes（节省进程、方便内存共享）
    #   lifecycle_nodes: 列表 = 需要由 lifecycle manager 管理的节点名（会传给 lifecycle_manager）
    #   remappings: 全局 remap（例如 /tf）以便 namespace 生效
    # 在文件后半部分：根据 use_composition 的值决定是用 GroupAction 启动独立节点，还是用 LoadComposableNodes 加载可组合节点。
    bringup_dir = get_package_share_directory('myagv_test_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # 中文说明：下面 7 个节点构成 Nav2 运行时导航链路，全部交给
    # lifecycle_manager_navigation 统一 configure/activate/deactivate/cleanup。
    # controller_server：
    #   局部控制服务器。订阅全局路径和当前速度/TF/局部代价地图，调用 controller 插件
    #   计算底盘速度。这里把内部输出 remap 到 cmd_vel_nav，后续再交给 velocity_smoother。
    # smoother_server：
    #   路径平滑服务器。接收 planner 输出的 Path，调用 smoother 插件做几何平滑或后处理，
    #   减少路径拐点、抖动和局部控制器跟踪压力。
    # planner_server：
    #   全局规划服务器。维护 global_costmap，按 planner_plugins 加载全局规划插件
    #   生成 map 坐标系下从当前位姿到目标位姿的全局路径。
    # behavior_server：
    #   恢复/行为服务器。加载 spin、backup、drive_on_heading、wait、assisted_teleop 等行为插件，
    #   在规划或控制失败时由行为树调用执行恢复动作。
    # bt_navigator：
    #   行为树导航调度器。对外提供 NavigateToPose/NavigateThroughPoses action，
    #   内部按 BT XML 编排 planner、controller、smoother、behavior 等服务调用。
    # waypoint_follower：
    #   路点跟随服务器。接收一组 waypoints，逐点调用 NavigateToPose，并可在每个路点执行
    #   wait/photo/input 等任务插件。
    # velocity_smoother：
    #   速度平滑器。接收 controller_server 输出的 cmd_vel_nav，按速度/加速度约束和反馈模式
    #   输出最终 cmd_vel，避免速度跳变直接作用到底盘。
    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        # default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            # 中文说明：smoother_server 是可选但常用的路径后处理节点，BT 中的 SmoothPath
            # action 会调用它；如果行为树不包含 SmoothPath，它仍可被其他客户端直接调用。
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # 中文说明：planner_server 承担全局路径生成，默认 GridBased 对应 NavfnPlanner；
            # planner_server 自己只负责 action/server/costmap/plugin 管理，不绑定具体算法。
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # 中文说明：behavior_server 承担恢复动作执行，BT Navigator 在失败分支调用这些行为，
            # 行为插件执行时通常依赖 local_costmap、TF 和 cmd_vel 输出能力。
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # 中文说明：bt_navigator 是导航任务入口，RViz 或上层系统发送 NavigateToPose action
            # 后，由它读取当前位姿、加载行为树，并串联规划、控制和恢复行为。
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # 中文说明：waypoint_follower 将多路点任务拆成多个 NavigateToPose 子任务，
            # 每个路点完成后可调用 waypoint_task_executor 插件执行停留、拍照、输入等任务。
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            # 中文说明：velocity_smoother 位于 controller_server 和底盘 cmd_vel 之间，
            # 输入 cmd_vel_nav，输出 cmd_vel，是实际底盘速度命令前的最后一道限幅/平滑。
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            # 中文说明：组合模式下以下组件加载到同一个 component_container，
            # 与独立进程模式职责完全一致，只是生命周期节点不再执行各自 main.cpp。
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
