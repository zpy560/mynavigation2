# 规控包主启动入口。按运行模式装配 Nav2、Collision Monitor、遥控与可视化节点，并统一参数、命名空间和生命周期。

import os
import yaml
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


# 装配三种运行模式的 Launch 描述；返回值仅描述启动动作，不会立即启动节点。
def resolve_braking_margin_arguments(context):
    with open(LaunchConfiguration('params_file').perform(context), encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    values = config.get('controller_server', {}).get('ros__parameters', {}).get('FixedPathController', {})
    defaults = {
        'goal_braking_distance_margin': 0.1,
        'dynamic_goal_braking_margin_enabled': False,
        'goal_braking_min_distance_margin': 0.03,
        'goal_braking_margin_transition_speed': 0.3,
    }
    actions = []
    for key, default in defaults.items():
        explicit = LaunchConfiguration('fixed_path_' + key).perform(context).strip()
        value = explicit if explicit else values.get(key, default)
        if isinstance(default, bool):
            value = str(value).lower()
            if value not in ('true', 'false'):
                raise ValueError('Invalid boolean fixed_path_' + key)
        else:
            value = str(float(value))
        actions.append(SetLaunchConfiguration('resolved_fixed_path_' + key, value))
    return actions


def start_velocity_diagnostics(context):
    if LaunchConfiguration('enable_velocity_diagnostics').perform(context).lower() != 'true':
        return []
    # 从同一参数文件解析实际反馈来源，不把控制器反馈误当作平滑器反馈。
    with open(LaunchConfiguration('params_file').perform(context), encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    namespace = LaunchConfiguration('namespace').perform(context)
    config = config.get(namespace, config) if namespace else config
    smoother = config.get('velocity_smoother', {}).get('ros__parameters', {})
    navigator = config.get('regulated_navigator', {}).get('ros__parameters', {})
    controller = config.get('controller_server', {}).get('ros__parameters', {})
    collision = LaunchConfiguration('use_collision_monitor').perform(context).lower() == 'true'
    return [Node(package='nav2_regulated_modules', executable='velocity_diagnostics_node',
                 name='velocity_diagnostics', output='both', parameters=[{
                     'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                     'operation_mode': LaunchConfiguration('operation_mode'),
                     'log_dir': LaunchConfiguration('log_dir'),
                     'velocity_file_log_frequency': ParameterValue(LaunchConfiguration('velocity_file_log_frequency'), value_type=float),
                     'velocity_console_log_frequency': ParameterValue(LaunchConfiguration('velocity_console_log_frequency'), value_type=float),
                     'use_collision_monitor': collision,
                     'smoother_output_topic': 'cmd_vel_collision_in' if collision else 'cmd_vel',
                     'smoother_feedback_topic': smoother.get('odom_topic', 'odom'),
                     'smoother_feedback_mode': smoother.get('feedback', 'OPEN_LOOP'),
                     'chassis_output_topic': navigator.get('chassis_output_topic', '/control_to_uart'),
                     'chassis_input_topic': navigator.get('chassis_input_topic', '/downstream/chassis_control'),
                     'controller_feedback_topic': controller.get('odom_topic', 'odom'),
                 }])]


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_regulated_modules')
    launch_dir = os.path.dirname(__file__)

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_collision_monitor = LaunchConfiguration('use_collision_monitor')
    use_collision_visualization = LaunchConfiguration('use_collision_visualization')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    operation_mode = LaunchConfiguration('operation_mode')
    adaptive_goal_braking_enabled = LaunchConfiguration('adaptive_goal_braking_enabled')
    enable_localization_jump_detection = LaunchConfiguration('enable_localization_jump_detection')
    fixed_path_progress_timeout = LaunchConfiguration('fixed_path_progress_timeout')
    default_keyboard_input_device = (os.ttyname(sys.stdin.fileno()) if sys.stdin.isatty() else '/dev/tty')
    keyboard_input_device = LaunchConfiguration('keyboard_input_device')
    # 遥控模式只启用遥控相关动作，不启动自主导航任务链。
    is_remote = PythonExpression(["'", operation_mode, "' == 'remote'"])
    # 自主或固定路径模式启用规划、跟踪及规控节点。
    is_navigation = PythonExpression(["'", operation_mode, "' != 'remote'"])
    # 启用碰撞监控时把平滑速度送入监控输入，否则直接送到 cmd_vel。
    smoothed_cmd_vel_topic = PythonExpression(["'cmd_vel_collision_in' if '", use_collision_monitor,"'.lower() == 'true' else 'cmd_vel'"])
    # 固定路径采用独立进度超时，其余模式保持普通导航超时。
    effective_progress_timeout = ParameterValue(
        PythonExpression([fixed_path_progress_timeout, " if '", operation_mode,"' == 'fixed_path' else 10.0"]),value_type=float)
    adaptive_fixed_path = PythonExpression(["'", operation_mode, "' == 'fixed_path' and '", adaptive_goal_braking_enabled, "'.lower() == 'true'"])
    # 仅在显式启用自适应固定路径时约束正常减速；默认保持原速度平滑器参数。
    fixed_path_smoother_deceleration = ParameterValue([
        PythonExpression(["-1.0 if (", adaptive_fixed_path, ") else -2.0"]),
        0.0, -3.2], value_type=list[float])
    fixed_path_immediate_stop = ParameterValue(
        adaptive_fixed_path, value_type=bool)
    fixed_path_smoother_params = {
        'max_decel': fixed_path_smoother_deceleration,
        'immediate_stop_on_zero_command': fixed_path_immediate_stop,
    }

    # 普通导航必需的生命周期节点列表；按依赖顺序交给 Lifecycle Manager 激活。
    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'smoother_server',
        'velocity_smoother',
        'regulated_navigator',
    ]

    remappings = [('/tf', 'tf'),('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'yaml_filename': map_yaml_file,
        'controller_server.ros__parameters.FixedPathController.adaptive_goal_braking_enabled': adaptive_goal_braking_enabled,
        **{'controller_server.ros__parameters.FixedPathController.' + key:
           LaunchConfiguration('resolved_fixed_path_' + key) for key in
           ['goal_braking_distance_margin', 'dynamic_goal_braking_margin_enabled',
            'goal_braking_min_distance_margin', 'goal_braking_margin_transition_speed']},
    }

    # 用启动实参重写 YAML 中的时钟、自动激活和地图路径，并保留 ROS 参数类型。
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    spdlog_log_dir_envvar = SetEnvironmentVariable('SPDLOG_WRAPPER_LOG_DIR', LaunchConfiguration('log_dir'))

    spdlog_console_level_envvar = SetEnvironmentVariable('SPDLOG_WRAPPER_CONSOLE_LEVEL', 'info')

    spdlog_file_level_envvar = SetEnvironmentVariable('SPDLOG_WRAPPER_FILE_LEVEL', EnvironmentVariable('SPDLOG_WRAPPER_FILE_LEVEL', default_value='trace'))

    spdlog_flush_interval_envvar = SetEnvironmentVariable('SPDLOG_WRAPPER_FLUSH_INTERVAL_SECONDS', '1')

    # 声明顶层命名空间；空字符串表示默认全局命名空间。
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')

    # 控制是否将规控节点放入指定命名空间。
    declare_use_namespace_cmd = DeclareLaunchArgument('use_namespace', default_value='False', description='Whether to apply a namespace to the navigation stack')

    # 指定地图文件；默认读取包内地图资源。
    declare_map_yaml_cmd = DeclareLaunchArgument('map', default_value=os.path.join('/userdata/map/pbstream', 'out.yaml'),description='Full path to map yaml file to load')

    # 决定是否使用仿真时钟，影响 TF 和超时判定。
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    # 指定规控栈参数 YAML；后续由 RewrittenYaml 注入启动实参。
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'regulated_modules.yaml'),
        description='Full path to the ROS 2 parameters file')

    # 指定 RViz 显示配置文件。
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        # default_value=os.path.join(bringup_dir, 'rviz', 'myrviz2.rviz'),
        description='Full path to the RViz config file')

    # 按需启动 RViz，不影响导航算法节点。
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz')

    # 决定是否让速度经过 Collision Monitor；同时影响速度话题路由。
    declare_use_collision_monitor_cmd = DeclareLaunchArgument(
        'use_collision_monitor',
        default_value='false',#'false true', #ooii
        description='Whether to start Collision Monitor and insert it into the velocity chain')

    # 默认启动碰撞区域 Marker 可视化，仍可通过启动参数关闭。
    declare_use_collision_visualization_cmd = DeclareLaunchArgument(
        'use_collision_visualization',
        default_value='true',#'false', #ooii
        description='Whether to start the Collision Monitor boundary visualizer')

    # 控制 Lifecycle Manager 是否自动激活受管节点。
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup lifecycle nodes')

    # 选择容器内组件加载或独立进程启动。
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if true')

    # 组合模式使用的组件容器名称。
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_regulated_container',
        description='Container name used when composition is enabled')

    # 独立进程异常退出时是否重启。
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Respawn nodes if they crash when composition is disabled')

    # 向各 Nav2 节点传递日志级别。
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level')

    # 选择 remote、autonomous 或 fixed_path；模式决定启动节点集合。
    declare_operation_mode_cmd = DeclareLaunchArgument(
        'operation_mode',
        default_value='autonomous',
        choices=['remote', 'autonomous', 'fixed_path'],
        description='Robot operation mode')

    declare_adaptive_goal_braking_cmd = DeclareLaunchArgument(
        'adaptive_goal_braking_enabled',
        default_value='false',
        description='Opt in to unvalidated adaptive fixed-path goal braking')

    # 控制定位突跳停车检测；TF 丢失保护不受此开关影响。
    declare_enable_localization_jump_detection_cmd = DeclareLaunchArgument(
        'enable_localization_jump_detection',
        default_value='false',
        description='Whether localization pose jumps cancel control and stop the robot')

    # 配置固定路径无进展的独立超时值，单位秒。
    declare_fixed_path_progress_timeout_cmd = DeclareLaunchArgument(
        'fixed_path_progress_timeout',
        default_value='120.0',
        description='Maximum stationary time allowed in fixed_path mode')

    # 指定键盘遥控设备；无交互终端时使用 /dev/tty。
    declare_keyboard_input_device_cmd = DeclareLaunchArgument(
        'keyboard_input_device',
        default_value=default_keyboard_input_device,
        description='Terminal device used by remote keyboard control')

    # 按 use_rviz 条件引入独立 RViz 启动文件。
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

    # 独立进程模式启动地图服务节点。
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='both',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings)

    # 管理地图服务的生命周期并等待地图就绪。
    lifecycle_manager_map_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='both',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server']}])

    # 独立进程模式下按条件启动导航、平滑和监控节点。
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='both',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='both',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {'progress_checker.movement_time_allowance':
                     effective_progress_timeout}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='both',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                condition=UnlessCondition(adaptive_fixed_path),
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='both',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                [('cmd_vel', 'cmd_vel_nav'),
                 ('cmd_vel_smoothed', smoothed_cmd_vel_topic)]),
            Node(
                condition=IfCondition(adaptive_fixed_path),
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='both',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, fixed_path_smoother_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                [('cmd_vel', 'cmd_vel_nav'),
                 ('cmd_vel_smoothed', smoothed_cmd_vel_topic)]),
            Node(
                condition=IfCondition(use_collision_monitor),
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='both',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {
                        'cmd_vel_in_topic': 'cmd_vel_collision_in',
                        'cmd_vel_out_topic': 'cmd_vel',
                    },
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_regulated_modules',
                executable='regulated_navigator_node',
                name='regulated_navigator',
                output='both',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {
                        'operation_mode': operation_mode,
                        'enable_localization_jump_detection': ParameterValue(
                            enable_localization_jump_detection,
                            value_type=bool),
                        'progress_timeout': effective_progress_timeout,
                    },
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='controlpub',
                executable='controlpub_node',
                name='controlpub',
                output='both',
                parameters=[{'input_topic': '/cmd_vel'},
                            {'output_topic': '/control_to_uart'}]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_regulated_modules',
                output='both',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
            Node(
                condition=IfCondition(use_collision_monitor),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_collision_monitor',
                output='both',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': ['collision_monitor']}]),
        ])

    # 组合模式复用其余组件，只为显式启用的新制动策略替换速度平滑器描述。
    composable_node_descriptions = [
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[
                    configured_params,
                    {'progress_checker.movement_time_allowance':
                     effective_progress_timeout}],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                [('cmd_vel', 'cmd_vel_nav'),
                 ('cmd_vel_smoothed', smoothed_cmd_vel_topic)]),
            ComposableNode(
                condition=IfCondition(use_collision_monitor),
                package='nav2_collision_monitor',
                plugin='nav2_collision_monitor::CollisionMonitor',
                name='collision_monitor',
                parameters=[
                    configured_params,
                    {
                        'cmd_vel_in_topic': 'cmd_vel_collision_in',
                        'cmd_vel_out_topic': 'cmd_vel',
                    },
                ],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_regulated_modules',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
            ComposableNode(
                condition=IfCondition(use_collision_monitor),
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_collision_monitor',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': ['collision_monitor']}]),
        ]

    adaptive_composable_node_descriptions = composable_node_descriptions.copy()
    adaptive_composable_node_descriptions[3] = ComposableNode(
        package='nav2_velocity_smoother',
        plugin='nav2_velocity_smoother::VelocitySmoother',
        name='velocity_smoother',
        parameters=[configured_params, fixed_path_smoother_params],
        remappings=remappings +
        [('cmd_vel', 'cmd_vel_nav'),
         ('cmd_vel_smoothed', smoothed_cmd_vel_topic)])
    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(PythonExpression(["'", use_composition, "'.lower() == 'true' and not (", adaptive_fixed_path, ")"])),
        target_container=container_name_full,
        composable_node_descriptions=composable_node_descriptions)
    load_adaptive_composable_nodes = LoadComposableNodes(
        condition=IfCondition(PythonExpression(["'", use_composition, "'.lower() == 'true' and (", adaptive_fixed_path, ")"])),
        target_container=container_name_full,
        composable_node_descriptions=adaptive_composable_node_descriptions)

    # 启动本包规控协调节点并传入任务模式、超时与定位参数。
    start_regulated_navigator_cmd = Node(
        condition=IfCondition(use_composition),
        package='nav2_regulated_modules',
        executable='regulated_navigator_node',
        name='regulated_navigator',
        output='both',
        parameters=[
            configured_params,
            {
                'operation_mode': operation_mode,
                'enable_localization_jump_detection': ParameterValue(
                    enable_localization_jump_detection,
                    value_type=bool),
                'progress_timeout': effective_progress_timeout,
            },
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings)

    # 启动底盘控制出口，把内部控制命令转换为串口输出。
    start_controlpub_cmd = Node(
        condition=IfCondition(use_composition),
        package='controlpub',
        executable='controlpub_node',
        name='controlpub',
        output='both',
        parameters=[{'input_topic': '/cmd_vel'},
                    {'output_topic': '/control_to_uart'}])

    ## ooii
    # 外部可视化按话题动态分配 Marker；默认订阅 Stop、Slowdown 和 L1。
    collision_boundary_visualizer_cmd = Node(
        condition=IfCondition(use_collision_visualization),
        package='nav2_regulated_modules',
        executable='collision_boundary_visualizer_node',
        name='collision_boundary_visualizer',
        output='both',
        parameters=[{'polygon_topics': [
            'collision_stop_zone',
            'collision_slowdown_zone',
            'collision_slowdown_l1',
        ]}])

    # 遥控模式单独启动键盘控制节点。
    remote_control_cmd = Node(
        condition=IfCondition(is_remote),
        package='myagv_keyboard_control',
        executable='myagv_keyboard_control_node',
        name='myagv_keyboard_control',
        output='both',
        emulate_tty=True,
        parameters=[configured_params, {'input_device': keyboard_input_device,
                     'output_topic': '/control_to_uart'}])

    # 把导航相关节点及可选 Collision Monitor 组合成模式受控的启动组。
    navigation_group = GroupAction(
        condition=IfCondition(is_navigation),
        actions=[
            rviz_cmd,
            map_server_cmd,
            lifecycle_manager_map_cmd,
            load_nodes,
            load_composable_nodes,
            load_adaptive_composable_nodes,
            start_regulated_navigator_cmd,
            start_controlpub_cmd,
            collision_boundary_visualizer_cmd,
        ])

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('enable_velocity_diagnostics', default_value='true'))
    ld.add_action(DeclareLaunchArgument('log_dir', default_value=EnvironmentVariable('SPDLOG_WRAPPER_LOG_DIR', default_value='/tmp/nav2_logs')))
    ld.add_action(DeclareLaunchArgument('velocity_file_log_frequency', default_value='100.0'))
    ld.add_action(DeclareLaunchArgument('velocity_console_log_frequency', default_value='1.0'))
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(spdlog_log_dir_envvar)
    ld.add_action(spdlog_console_level_envvar)
    ld.add_action(spdlog_file_level_envvar)
    ld.add_action(spdlog_flush_interval_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    for key in ['goal_braking_distance_margin', 'dynamic_goal_braking_margin_enabled',
                'goal_braking_min_distance_margin', 'goal_braking_margin_transition_speed']:
        ld.add_action(DeclareLaunchArgument('fixed_path_' + key, default_value='',
                                           description='Explicit FixedPathController override; empty uses parameter YAML'))
    ld.add_action(OpaqueFunction(function=resolve_braking_margin_arguments))
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_collision_monitor_cmd)
    ld.add_action(declare_use_collision_visualization_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_operation_mode_cmd)
    ld.add_action(declare_adaptive_goal_braking_cmd)
    ld.add_action(declare_enable_localization_jump_detection_cmd)
    ld.add_action(declare_fixed_path_progress_timeout_cmd)
    ld.add_action(declare_keyboard_input_device_cmd)
    ld.add_action(OpaqueFunction(function=start_velocity_diagnostics))
    ld.add_action(remote_control_cmd)
    ld.add_action(navigation_group)

    return ld
