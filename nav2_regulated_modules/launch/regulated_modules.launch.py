# 三模式规控的 Launch 装配：解析实参、重写 YAML、选择节点集合，再交给生命周期管理器激活。
# LaunchConfiguration 是延迟求值对象；OpaqueFunction 在上下文可用后读取真实参数，不能在生成描述时当普通字符串使用。
# 当前模式仍互斥；启用开关、参数文件值和 C++ 回退默认需分层解释。

import os
import yaml
import sys
import math

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


class NavigationRewrittenYaml(RewrittenYaml):
    """Apply mode-specific motion profiles without changing the shared YAML."""

    def __init__(self, **kwargs):
        self.navigation_kwargs = kwargs
        super().__init__(**kwargs)

    def perform(self, context):
        if (LaunchConfiguration('operation_mode').perform(context) == 'fixed_path'
                and LaunchConfiguration('adaptive_goal_braking_enabled', default='false').perform(context).lower() != 'true'):
            return self.loaded_fixed_path_parameters(context)
        if LaunchConfiguration('operation_mode').perform(context) == 'autonomous_fixed':
            return self.fixed_parameters(context)
        if LaunchConfiguration('operation_mode').perform(context) != 'autonomous':
            return super().perform(context)
        rewrites = dict(self.navigation_kwargs['param_rewrites'])
        rewrites.update({
            'controller_server.ros__parameters.controller_frequency': '50.0',
            'controller_server.ros__parameters.MPPI.model_dt': '0.02',
            'controller_server.ros__parameters.MPPI.time_steps': '75',
            'controller_server.ros__parameters.MPPI.vx_max': '0.5',
            'controller_server.ros__parameters.MPPI.visualize': 'true',
            'controller_server.ros__parameters.MPPI.TrajectoryVisualizer.trajectory_step': '15',
            'controller_server.ros__parameters.MPPI.TrajectoryVisualizer.time_step': '5',
            'velocity_smoother.ros__parameters.smoothing_frequency': '50.0',
            'controller_server.ros__parameters.progress_checker.plugin': 'nav2_controller::PoseProgressChecker',
            'controller_server.ros__parameters.progress_checker.required_movement_radius': '0.05',
            'controller_server.ros__parameters.progress_checker.required_movement_angle': '0.2',
            'regulated_navigator.ros__parameters.progress_min_translation': '0.05',
        })
        autonomous_planner_id = LaunchConfiguration('autonomous_planner_id', default='SmacHybrid').perform(context).strip()
        if autonomous_planner_id:
            rewrites['regulated_navigator.ros__parameters.planner_id'] = autonomous_planner_id
            # PlannerServer prioritizes selected_planner over the Action request.
            rewrites['planner_server.ros__parameters.selected_planner'] = autonomous_planner_id
        if LaunchConfiguration('autonomous_mppi_profile', default='false').perform(context).lower() == 'true':
            source = perform_substitutions(context, normalize_to_list_of_substitutions(self.navigation_kwargs['source_file']))
            with open(source, encoding='utf-8') as stream:
                config = yaml.safe_load(stream) or {}
            namespace = perform_substitutions(context, normalize_to_list_of_substitutions(self.navigation_kwargs.get('root_key') or ''))
            config = config.get(namespace, config) if namespace else config
            xy_tolerance = float(config['controller_server']['ros__parameters']['stopped_goal_checker']['xy_goal_tolerance'])
            profile = {
                'plugin': 'nav2_regulated_modules::AutonomousMPPIController',
                'primary_controller': 'nav2_mppi_controller::MPPIController',
                'batch_size': '300',
                'angular_dist_threshold': '0.7853981633974483',
                'angular_disengage_threshold': '0.17453292519943295',
                'forward_sampling_distance': '0.5',
                'rotate_to_heading_angular_vel': '0.5',
                'max_angular_accel': '0.8',
                'simulate_ahead_time': '0.5',
                'rotate_to_goal_heading': 'true',
                'closed_loop': 'true',
                'autonomous_approach_distance': '1.0',
                'autonomous_terminal_distance': '0.15',
                'autonomous_terminal_max_linear_velocity': '0.05',
                'autonomous_terminal_max_angular_velocity': '0.1',
                'GoalAngleCritic.enabled': 'false',
            }
            for key in ['autonomous_terminal_distance', 'autonomous_terminal_max_linear_velocity', 'autonomous_terminal_max_angular_velocity']:
                profile[key] = str(config['controller_server']['ros__parameters'].get('MPPI', {}).get(key, profile[key]))
            rewrites.update({'controller_server.ros__parameters.MPPI.' + key: value for key, value in profile.items()})
            rewrites.update({
                'controller_server.ros__parameters.stopped_goal_checker.plugin': 'nav2_regulated_modules::AutonomousStoppedGoalChecker',
                'controller_server.ros__parameters.stopped_goal_checker.stateful': 'false',
                'controller_server.ros__parameters.stopped_goal_checker.settle_duration': '0.5',
                'regulated_navigator.ros__parameters.autonomous_terminal_progress_enabled': 'true',
                'regulated_navigator.ros__parameters.autonomous_terminal_xy_tolerance': str(xy_tolerance),
                'regulated_navigator.ros__parameters.autonomous_terminal_distance': profile['autonomous_terminal_distance'],
            })
        return RewrittenYaml(**{**self.navigation_kwargs, 'param_rewrites': rewrites}).perform(context)

    def loaded_fixed_path_parameters(self, context):
        """Bound ordinary fixed-path longitudinal motion throughout the task."""
        import tempfile
        baseline = super().perform(context)
        with open(baseline, encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        namespace = perform_substitutions(context, normalize_to_list_of_substitutions(self.navigation_kwargs.get('root_key') or ''))
        config = document[namespace] if namespace else document
        smoother = config['velocity_smoother']['ros__parameters']
        # Preserve lateral/angular limits and controller braking geometry; only the x axis changes.
        limits = {
            'max_accel': (0.3, [2.0, 0.0, 3.2]),
            'max_decel': (-0.3, [-2.0, 0.0, -3.2]),
            'max_accel_jerk': (1.0, [6.0, 0.0, 4.0]),
            'max_decel_jerk': (1.0, [8.0, 0.0, 6.0]),
        }
        for name, (longitudinal, default) in limits.items():
            values = list(smoother.get(name, default))
            if len(values) != 3:
                raise ValueError(f'{name} must contain x, y and angular limits')
            current = float(values[0])
            if not math.isfinite(current) or current * longitudinal <= 0.0:
                raise ValueError(f'{name} must have a finite x limit with the correct sign')
            values[0] = max(current, longitudinal) if longitudinal < 0.0 else min(current, longitudinal)
            smoother[name] = values
        # The existing S-curve uses the smaller acceleration/deceleration magnitude for both.
        effective_deceleration = min(smoother['max_accel'][0], -smoother['max_decel'][0])
        planned_deceleration = float(config['controller_server']['ros__parameters']['FixedPathController'].get('goal_linear_deceleration', 0.25))
        if not math.isfinite(planned_deceleration) or planned_deceleration <= 0.0 or planned_deceleration > effective_deceleration:
            raise ValueError('fixed_path goal_linear_deceleration must be positive and no greater than the effective longitudinal S-curve limit')
        output = tempfile.NamedTemporaryFile(mode='w', delete=False)
        yaml.safe_dump(document, output)
        output.close()
        return output.name

    def fixed_parameters(self, context):
        """Fourth-mode profile; never rewrite the three existing mode profiles."""
        import tempfile
        baseline = super().perform(context)
        with open(baseline, encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        namespace = perform_substitutions(context, normalize_to_list_of_substitutions(self.navigation_kwargs.get('root_key') or ''))
        config = document[namespace] if namespace else document
        # Only this mode uses the requested 0.8 x 0.6 m envelope and wider obstacle cost field.
        footprint = '[[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]]'
        for name in ['global_costmap', 'local_costmap']:
            costmap = config[name][name]['ros__parameters']
            costmap.update(footprint=footprint, footprint_padding=0.0)
            costmap.pop('robot_radius', None)
            costmap['inflation_layer'].update(inflation_radius=0.90, cost_scaling_factor=2.5)
        navigator = config['regulated_navigator']['ros__parameters']
        # Budget the error against the original action goal, including planner quantization.
        final_xy = float(navigator.get('autonomous_fixed_xy_tolerance', 0.01))
        if not 0.0 < final_xy < float('inf'):
            raise ValueError('autonomous_fixed_xy_tolerance must be finite and positive')
        final_xy = min(final_xy, 0.01)
        endpoint_budget = min(0.003, final_xy * 0.3)
        speed = float(navigator.get('autonomous_fixed_max_speed', 0.5))
        short_pause_timeout = float(navigator.get('autonomous_fixed_short_pause_timeout', 0.5))
        resume_stable_duration = float(navigator.get('autonomous_fixed_resume_stable_duration', 0.15))
        localization_timeout = float(navigator.get('localization_timeout', 0.3))
        tracking_localization_timeout = float(navigator.get('autonomous_fixed_tracking_localization_timeout', max(0.6, localization_timeout)))
        terminal_localization_distance = float(navigator.get('autonomous_fixed_terminal_localization_distance', 0.5))
        if (not all(math.isfinite(value) and value > 0.0 for value in
                    [localization_timeout, tracking_localization_timeout, terminal_localization_distance])
                or tracking_localization_timeout < localization_timeout):
            raise ValueError('autonomous_fixed localization timeouts and terminal distance must be positive; tracking timeout must cover the strict timeout')
        navigator.update(operation_mode='autonomous_fixed', planner_id='SmacHybrid',
                         controller_id='AutonomousFixedPathController',
                         goal_checker_id='autonomous_fixed_goal_checker', use_smoother=False,
                         progress_timeout=10.0, replan_frequency=1.0,
                         autonomous_fixed_xy_tolerance=final_xy,
                         autonomous_fixed_short_pause_timeout=short_pause_timeout,
                         autonomous_fixed_resume_stable_duration=resume_stable_duration,
                         autonomous_fixed_tracking_localization_timeout=tracking_localization_timeout,
                         autonomous_fixed_terminal_localization_distance=terminal_localization_distance)
        planner = config['planner_server']['ros__parameters']
        planner['planner_plugins'] = ['SmacHybrid']
        planner['selected_planner'] = 'SmacHybrid'
        planner['SmacHybrid'].update(plugin='nav2_regulated_modules::AutonomousFixedHybridPlanner',
                                    motion_model_for_search='REEDS_SHEPP', tolerance=endpoint_budget,
                                    minimum_turning_radius=0.40, max_planning_time=3.5, smooth_path=True,
                                    lookup_table_size=10.0)
        controller = config['controller_server']['ros__parameters']
        controller.update(controller_frequency=100.0,
                          controller_plugins=['AutonomousFixedPathController'],
                          goal_checker_plugins=['autonomous_fixed_goal_checker', 'autonomous_fixed_cusp_checker'])
        tracker = dict(controller['FixedPathController'])
        tracker.pop('plugin', None)
        tracker.update(desired_linear_vel=speed, rotate_to_heading_angular_vel=0.4,
                       max_angular_accel=0.8, terminal_convergence_enabled=True,
                       goal_final_approach_velocity=0.01,
                       goal_stop_entry_tolerance=final_xy * 0.5,
                       goal_terminal_lookahead_dist=0.08,
                       goal_terminal_lookahead_min_dist=0.05,
                       goal_terminal_lookahead_max_dist=0.12,
                       goal_terminal_lookahead_reference_speed=0.1,
                       goal_terminal_lookahead_speed_gain=0.2)
        controller['AutonomousFixedPathController'] = {
            'plugin': 'nav2_regulated_modules::AutonomousFixedPathController',
            'tracker': tracker, 'settle_duration': 0.5,
            'brake_stable_duration': 0.2, 'terminal_review_timeout': 2.0,
            'terminal_capture_margin': 0.02,
            'terminal_trim_max_speed': 0.02, 'terminal_trim_max_step': 0.02,
            'terminal_trim_max_distance': 0.03, 'terminal_trim_timeout': 5.0,
            'terminal_trim_max_attempts': 2, 'terminal_trim_confirm_duration': 0.2,
            'max_angular_velocity': 0.4, 'max_angular_acceleration': 0.8,
            'localization_timeout': localization_timeout,
            'tracking_localization_timeout': tracking_localization_timeout,
            'terminal_localization_distance': terminal_localization_distance,
            'short_pause_timeout': short_pause_timeout,
            'resume_stable_duration': resume_stable_duration,
            'resume_distance': float(navigator.get('autonomous_fixed_resume_distance', 0.2)),
            'resume_linear_acceleration': 0.5,
            'obstacle_timeout': float(navigator.get('autonomous_fixed_obstacle_timeout', 60.0)),
        }
        for name, xy, yaw in [('autonomous_fixed_goal_checker', final_xy - endpoint_budget, float(navigator.get('autonomous_fixed_yaw_tolerance', 0.15726646259971647)) * 0.5),
                              ('autonomous_fixed_cusp_checker', 0.01, 0.08726646259971647)]:
            controller[name] = {'plugin': 'nav2_controller::StoppedGoalChecker', 'stateful': False,
                                'xy_goal_tolerance': xy, 'yaw_goal_tolerance': yaw,
                                'trans_stopped_velocity': 0.01, 'rot_stopped_velocity': 0.05}
        controller['progress_checker'].update(plugin='nav2_controller::PoseProgressChecker',
                                               required_movement_radius=0.01,
                                               required_movement_angle=0.05,
                                               movement_time_allowance=120.0)
        config['velocity_smoother']['ros__parameters'].update(
            smoothing_frequency=100.0, max_velocity=[speed, 0.0, 0.4], min_velocity=[-speed, 0.0, -0.4])
        with tempfile.NamedTemporaryFile(mode='w', prefix='autonomous-fixed-', delete=False, encoding='utf-8') as stream:
            yaml.safe_dump(document, stream)
            return stream.name


# 仿真入口的显式覆盖由现有 RewrittenYaml 统一处理，不再生成额外参数文件。
FIXED_PATH_OVERRIDES = {
    'desired_linear_vel': ('fixed_path_max_linear_velocity', 1.5),
    'approach_velocity_scaling_dist': ('fixed_path_approach_velocity_scaling_dist', 0.8),
    'goal_linear_deceleration': ('fixed_path_goal_linear_deceleration', 0.25),
    'goal_final_approach_velocity': ('fixed_path_goal_final_approach_velocity', 0.01),
    'goal_braking_reaction_time': ('fixed_path_goal_braking_reaction_time', 0.1),
    'goal_terminal_lookahead_dist': ('fixed_path_goal_terminal_lookahead_dist', 0.2),
    'goal_terminal_lookahead_reference_speed': ('fixed_path_goal_terminal_lookahead_reference_speed', 0.75),
    'goal_terminal_lookahead_speed_gain': ('fixed_path_goal_terminal_lookahead_speed_gain', 0.1),
    'goal_terminal_lookahead_min_dist': ('fixed_path_goal_terminal_lookahead_min_dist', 0.15),
    'goal_terminal_lookahead_max_dist': ('fixed_path_goal_terminal_lookahead_max_dist', 0.25),
    'rotate_to_heading_angular_vel': ('fixed_path_rotate_to_heading_angular_vel', 0.4),
    'max_angular_accel': ('fixed_path_max_angular_accel', 0.8),
    'rotate_to_heading_kp': ('fixed_path_rotate_to_heading_kp', 1.5),
    'goal_error_log_frequency': ('fixed_path_goal_error_log_frequency', 1.0),
}


# 读取 YAML 和显式 Launch 覆盖，保留布尔／浮点类型并验证裕量关系。
# 空覆盖沿用参数文件；此处的解析结果用于后续重写，不会热改已经运行的节点。
def resolve_braking_margin_arguments(context):
    with open(LaunchConfiguration('params_file').perform(context), encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    values = config.get('controller_server', {}).get('ros__parameters', {}).get('FixedPathController', {})
    defaults = {
        'goal_braking_distance_margin': 0.1,
        'dynamic_goal_braking_margin_enabled': True,
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
    for key, (argument, default) in FIXED_PATH_OVERRIDES.items():
        explicit = LaunchConfiguration(argument, default='').perform(context).strip()
        value = float(explicit if explicit else values.get(key, default))
        actions.append(SetLaunchConfiguration('resolved_fixed_path_' + key, str(value)))
    controller = config.get('controller_server', {}).get('ros__parameters', {})
    navigator = config.get('regulated_navigator', {}).get('ros__parameters', {})
    mode = LaunchConfiguration('operation_mode', default='autonomous').perform(context)
    tolerance = LaunchConfiguration('fixed_path_xy_goal_tolerance', default='').perform(context).strip()
    for checker in ['fixed_path_goal_checker', 'stopped_goal_checker']:
        value = controller.get(checker, {}).get('xy_goal_tolerance', 0.01)
        if tolerance and (checker == 'fixed_path_goal_checker' or mode == 'fixed_path'):
            value = float(tolerance)
        actions.append(SetLaunchConfiguration('resolved_' + checker + '_xy_goal_tolerance', str(value)))
    use_smoother = LaunchConfiguration('use_path_smoother', default='').perform(context).strip().lower()
    if use_smoother and use_smoother not in ('true', 'false'):
        raise ValueError('Invalid boolean use_path_smoother')
    actions.append(SetLaunchConfiguration('resolved_use_path_smoother',
                   use_smoother or str(navigator.get('use_smoother', True)).lower()))
    return actions


# 按命名空间读取平滑器参数块的 collision 话题与超时，校验后统一提供给三节点。
# 超时必须是有限正数，不能只验证 YAML 中有这个键。
def resolve_collision_state_arguments(context):
    with open(LaunchConfiguration('params_file').perform(context), encoding='utf-8') as stream:
        config = yaml.safe_load(stream) or {}
    namespace = LaunchConfiguration('namespace').perform(context)
    config = config.get(namespace, config) if namespace else config
    values = config.get('velocity_smoother', {}).get('ros__parameters', {})
    topic = values.get('collision_state_topic', '/collision_state')
    timeout = float(values.get('collision_state_timeout', 0.5))
    if not isinstance(topic, str) or not topic.strip() or not math.isfinite(timeout) or timeout <= 0.0:
        raise ValueError('Invalid collision state topic or timeout')
    return [SetLaunchConfiguration('resolved_collision_state_topic', topic),
            SetLaunchConfiguration('resolved_collision_state_timeout', str(timeout))]


# 诊断关闭时返回空 Action 列表；开启时从同份 YAML 提取真实反馈话题。
# 只构造 Node 描述；是否真正启动由 Launch 执行阶段决定。
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


# 创建启动实参声明与条件 Action，并返回 LaunchDescription。
# 这里只描述启动关系；节点创建、参数替换与生命周期激活由 Launch 执行阶段完成。
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
    legacy_velocity_logging = PythonExpression([
        "'", LaunchConfiguration('enable_legacy_velocity_logs'), "'.lower() == 'true' or '",
        LaunchConfiguration('enable_velocity_diagnostics'), "'.lower() != 'true'"])
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
        PythonExpression([fixed_path_progress_timeout, " if '", operation_mode,"' == 'fixed_path' else (120.0 if '", operation_mode, "' == 'autonomous_fixed' else 10.0)"]),value_type=float)
    adaptive_fixed_path = PythonExpression(["'", operation_mode, "' == 'fixed_path' and '", adaptive_goal_braking_enabled, "'.lower() == 'true'"])
    # 显式自适应分支保留既有覆盖；默认 fixed_path 限值由 NavigationRewrittenYaml 设置。
    fixed_path_smoother_deceleration = ParameterValue([PythonExpression(["-1.0 if (", adaptive_fixed_path, ") else -2.0"]),0.0, -3.2], value_type=list[float])
    fixed_path_immediate_stop = ParameterValue(adaptive_fixed_path, value_type=bool)
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
        **{node + '.ros__parameters.legacy_velocity_logging': legacy_velocity_logging
           for node in ['regulated_navigator', 'velocity_smoother']},
        **{node + '.ros__parameters.' + key: value
           for node in ['controller_server', 'velocity_smoother', 'regulated_navigator']
           for key, value in {
               'collision_state_enabled': PythonExpression([
                   "'", operation_mode, "' in ('fixed_path', 'autonomous_fixed') and '",
                   LaunchConfiguration('enable_collision_state'), "'.lower() == 'true'"]),
               'collision_state_topic': LaunchConfiguration('resolved_collision_state_topic'),
               'collision_state_timeout': LaunchConfiguration('resolved_collision_state_timeout'),
           }.items()},
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'yaml_filename': map_yaml_file,
        'controller_server.ros__parameters.FixedPathController.adaptive_goal_braking_enabled': adaptive_goal_braking_enabled,
        **{'controller_server.ros__parameters.FixedPathController.' + key:
           LaunchConfiguration('resolved_fixed_path_' + key) for key in
           ['goal_braking_distance_margin', 'dynamic_goal_braking_margin_enabled',
            'goal_braking_min_distance_margin', 'goal_braking_margin_transition_speed']},
        **{'controller_server.ros__parameters.FixedPathController.' + key:
           LaunchConfiguration('resolved_fixed_path_' + key) for key in FIXED_PATH_OVERRIDES},
        **{'controller_server.ros__parameters.' + checker + '.xy_goal_tolerance':
           LaunchConfiguration('resolved_' + checker + '_xy_goal_tolerance')
           for checker in ['fixed_path_goal_checker', 'stopped_goal_checker']},
        'regulated_navigator.ros__parameters.use_smoother': LaunchConfiguration('resolved_use_path_smoother'),
    }

    # 用启动实参重写 YAML 中的时钟、自动激活和地图路径，并保留 ROS 参数类型。
    configured_params = ParameterFile(
        NavigationRewrittenYaml(
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

    # 三种导航模式分别提供独立配置；显式指定文件时沿用用户选择。
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=[os.path.join(bringup_dir, 'rviz'), '/', PythonExpression([
            "'", operation_mode, "' if '", operation_mode,
            "' in ('autonomous_fixed', 'autonomous', 'fixed_path') else 'nav2_default_view'"
        ]), '.rviz'],
        description='Full path to the RViz config file')

    # 导航默认由另一终端打开 RViz；遥控模式保留原默认值。
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value=PythonExpression([
            "'false' if '", operation_mode,
            "' in ('autonomous_fixed', 'autonomous', 'fixed_path') else 'true'"
        ]),
        description='Whether to start RViz')

    # 决定是否让速度经过 Collision Monitor；同时影响速度话题路由。
    declare_use_collision_monitor_cmd = DeclareLaunchArgument(
        'use_collision_monitor',
        default_value='false',
        description='Whether to start Collision Monitor and insert it into the velocity chain')

    # 默认启动碰撞区域 Marker 可视化，仍可通过启动参数关闭。
    declare_use_collision_visualization_cmd = DeclareLaunchArgument(
        'use_collision_visualization',
        default_value='true',# 导航模式的显示开关，不产生碰撞状态。
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
        choices=['remote', 'autonomous', 'fixed_path', 'autonomous_fixed'],
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
                            {'output_topic': '/control_to_uart',
                             'legacy_velocity_logging': ParameterValue(legacy_velocity_logging, value_type=bool)}]),
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

    # 组合部署使用以下组件列表；固定路径且自适应制动启用时，为平滑器追加 fixed_path_smoother_params。
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
                    {'output_topic': '/control_to_uart',
                     'legacy_velocity_logging': ParameterValue(legacy_velocity_logging, value_type=bool)}])

    # 边界显示独立于碰撞判定；按配置的多边形话题生成 Marker。
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
    ld.add_action(DeclareLaunchArgument('autonomous_planner_id', default_value='SmacHybrid',
                                       description='Autonomous planner ID; empty follows YAML, ignored outside autonomous mode'))
    ld.add_action(DeclareLaunchArgument('autonomous_mppi_profile', default_value='false',
                                       description='Use the simulation autonomous MPPI wrapper; ignored outside autonomous mode'))
    ld.add_action(DeclareLaunchArgument('enable_legacy_velocity_logs', default_value='false'))
    ld.add_action(DeclareLaunchArgument('log_dir', default_value=EnvironmentVariable('SPDLOG_WRAPPER_LOG_DIR', default_value='/tmp/nav2_logs')))
    ld.add_action(DeclareLaunchArgument('velocity_file_log_frequency', default_value=PythonExpression(["'50.0' if '", LaunchConfiguration('operation_mode', default='autonomous'), "' == 'autonomous' else '100.0'"])))
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
    ld.add_action(DeclareLaunchArgument('enable_collision_state', default_value='true',
                                      description='Enable fixed-path collision state gating before S-curve smoothing (default on)'))
    ld.add_action(OpaqueFunction(function=resolve_collision_state_arguments))
    for key in ['goal_braking_distance_margin', 'dynamic_goal_braking_margin_enabled',
                'goal_braking_min_distance_margin', 'goal_braking_margin_transition_speed']:
        ld.add_action(DeclareLaunchArgument('fixed_path_' + key, default_value='',
                                           description='Explicit FixedPathController override; empty uses parameter YAML'))
    for argument, _ in FIXED_PATH_OVERRIDES.values():
        ld.add_action(DeclareLaunchArgument(argument, default_value='',
                                           description='Explicit FixedPathController override; empty uses parameter YAML'))
    ld.add_action(DeclareLaunchArgument('fixed_path_xy_goal_tolerance', default_value='',
                                       description='Position tolerance override; empty uses parameter YAML'))
    ld.add_action(DeclareLaunchArgument('use_path_smoother', default_value='',
                                       description='Override navigator path smoothing; empty uses parameter YAML'))
    ld.add_action(declare_operation_mode_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_collision_monitor_cmd)
    ld.add_action(declare_use_collision_visualization_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(OpaqueFunction(function=resolve_braking_margin_arguments))
    ld.add_action(declare_adaptive_goal_braking_cmd)
    ld.add_action(declare_enable_localization_jump_detection_cmd)
    ld.add_action(declare_fixed_path_progress_timeout_cmd)
    ld.add_action(declare_keyboard_input_device_cmd)
    ld.add_action(OpaqueFunction(function=start_velocity_diagnostics))
    ld.add_action(remote_control_cmd)
    ld.add_action(navigation_group)

    ld.add_action(LogInfo(condition=IfCondition(PythonExpression(["'", operation_mode, "' == 'autonomous'"])), msg=['Autonomous rates: controller=50Hz, smoother=50Hz, diagnostic=', LaunchConfiguration('velocity_file_log_frequency'), 'Hz, console=', LaunchConfiguration('velocity_console_log_frequency'), 'Hz; MPPI model_dt=0.02s/time_steps=75']))
    return ld
