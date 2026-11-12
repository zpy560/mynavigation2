#!/usr/bin/env python3
"""Real navigator with synthetic planner/controller/TF in an isolated ROS domain.

This verifies orchestration only, not Hybrid search, controller quality or physical arrival.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Path as NavPath
from byd_custom_msgs.msg import MotionState
from nav2_msgs.action import ComputePathToPose, ComputePathThroughPoses, FollowPath, NavigateToPose
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class Driver(Node):
    def __init__(self):
        super().__init__('autonomous_fixed_protocol_driver')
        self.x = 0.0
        self.emit_tf = True
        self.block = False
        self.fail_plan = False
        self.fail_control = False
        self.cusp = False
        self.downstream = False
        self.running = True
        self.phase_override = None
        self.suppress_status = False
        self.zero_commands = 0
        self.plans = 0
        self.follows = 0
        self.frozen = []
        self.group = ReentrantCallbackGroup()
        self.tf = TransformBroadcaster(self)
        self.motion = self.create_publisher(MotionState, '/motion_state', 10)
        self.status = self.create_publisher(String, 'autonomous_fixed/execution_status', 10)
        self.raw = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.final = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self.observe_command, 10)
        self.create_subscription(NavPath, 'autonomous_fixed/plan', lambda message: self.frozen.append(message), 10)
        self.create_timer(0.02, self.tick, callback_group=self.group)
        self.planner = ActionServer(self, ComputePathToPose, 'compute_path_to_pose', self.plan, callback_group=self.group)
        self.through = ActionServer(self, ComputePathThroughPoses, 'compute_path_through_poses', self.unused, callback_group=self.group)
        self.controller = ActionServer(self, FollowPath, 'follow_path', self.follow,
                                       cancel_callback=lambda goal: CancelResponse.ACCEPT, callback_group=self.group)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.group)

    def observe_command(self, message):
        if message.linear.x == 0.0 and message.angular.z == 0.0:
            self.zero_commands += 1

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        if self.emit_tf:
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'base_link'
            transform.header.stamp = stamp
            transform.transform.translation.x = self.x
            transform.transform.rotation.w = 1.0
            self.tf.sendTransform(transform)
        motion = MotionState()
        motion.header.stamp = stamp
        motion.header.frame_id = 'base_link'
        self.motion.publish(motion)

    def unused(self, goal):
        raise AssertionError('Multi-point planner must not be called')

    def plan(self, goal):
        self.plans += 1
        result = ComputePathToPose.Result()
        if self.fail_plan:
            goal.abort()
            return result
        result.path.header.frame_id = 'map'
        positions = [self.x + (goal.request.goal.pose.position.x - self.x) * index / 8 for index in range(9)]
        if self.cusp:
            positions = [self.x, self.x + 0.2, self.x - 0.2, goal.request.goal.pose.position.x]
        for position in positions:
            point = PoseStamped()
            point.header.frame_id = 'map'
            point.pose.position.x = position
            point.pose.orientation.w = 1.0
            result.path.poses.append(point)
        goal.succeed()
        return result

    def follow(self, goal):
        self.follows += 1
        if self.fail_control:
            goal.abort()
            return FollowPath.Result()
        path = goal.request.path
        execution = path.header.stamp.sec * 10**9 + path.header.stamp.nanosec
        while self.running:
            if goal.is_cancel_requested:
                goal.canceled()
                return FollowPath.Result()
            if not self.block and self.emit_tf and not self.phase_override:
                error = path.poses[-1].pose.position.x - self.x
                self.x += max(-0.003, min(0.003, error))
            index = min(range(len(path.poses)), key=lambda i: abs(path.poses[i].pose.position.x - self.x))
            raw = Twist()
            raw.linear.x = 0.0 if self.phase_override else 0.1
            self.raw.publish(raw)
            self.final.publish(Twist() if self.block else raw)
            phase = self.phase_override or ('WAIT_OBSTACLE' if self.block and not self.downstream else 'TRACKING')
            if not self.suppress_status:
                self.status.publish(String(data=json.dumps({'execution_id': execution, 'point_index': index, 'phase': phase}, separators=(',', ':'))))
            if abs(path.poses[-1].pose.position.x - self.x) < 1e-6 and not self.block:
                time.sleep(0.15)
                goal.succeed()
                return FollowPath.Result()
            time.sleep(0.02)
        goal.abort()
        return FollowPath.Result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0 or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('An explicit nonzero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1 are required')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    driver = Driver()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(driver)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    prefix = Path(get_package_prefix('nav2_regulated_modules'))
    log = (output / 'navigator.log').open('w')
    parameters = {'operation_mode': 'autonomous_fixed', 'planner_id': 'SmacHybrid',
                  'velocity_odom_topic': '/motion_state',
                  'use_smoother': 'false', 'legacy_velocity_logging': 'false',
                  'collision_state_enabled': 'false', 'progress_timeout': '1.0',
                  'localization_timeout': '0.15', 'localization_stable_duration': '0.15',
                  'localization_recovery_timeout': '2.0', 'autonomous_fixed_obstacle_timeout': '2.0'}
    command = [str(prefix / 'lib/nav2_regulated_modules/regulated_navigator_node'), '--ros-args']
    for name, value in parameters.items():
        command += ['-p', f'{name}:={value}']
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    report = {'kind': 'Synthetic isolated protocol test, not vehicle/control-performance evidence', 'cases': []}

    def wait(predicate, timeout=20):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        raise AssertionError('Protocol condition timed out')

    def result(future):
        wait(future.done)
        return future.result()

    def transition(value):
        client = driver.create_client(ChangeState, '/regulated_navigator/change_state')
        wait(client.service_is_ready)
        request = ChangeState.Request()
        request.transition.id = value
        assert result(client.call_async(request)).success

    def start():
        driver.block = False
        driver.emit_tf = True
        driver.phase_override = None
        driver.suppress_status = False
        plans, follows, frozen = driver.plans, driver.follows, len(driver.frozen)
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = 'map'
        request.pose.pose.position.x = driver.x + 0.4
        request.pose.pose.orientation.w = 1.0
        handle = result(driver.client.send_goal_async(request))
        assert handle.accepted
        return handle, handle.get_result_async(), plans, follows, frozen

    try:
        transition(Transition.TRANSITION_CONFIGURE)
        transition(Transition.TRANSITION_ACTIVATE)
        wait(lambda: driver.client.server_is_ready())
        for scenario in ['normal', 'multi_cusp', 'navigator_tf_delay', 'localization_short_pause', 'localization_resume', 'pause_timeout', 'paused_status_timeout', 'obstacle_resume', 'downstream_stop_resume', 'obstacle_timeout', 'planning_failure', 'control_failure', 'cancel']:
            driver.fail_plan = scenario == 'planning_failure'
            driver.fail_control = scenario == 'control_failure'
            driver.cusp = scenario == 'multi_cusp'
            driver.downstream = scenario == 'downstream_stop_resume'
            handle, done, plans, follows, frozen = start()
            if scenario not in ['planning_failure', 'control_failure']:
                wait(lambda: driver.follows > follows)
            if scenario == 'navigator_tf_delay':
                time.sleep(0.1)
                zero_commands = driver.zero_commands
                driver.emit_tf = False
                time.sleep(0.4)
                assert driver.follows - follows == 1 and driver.zero_commands == zero_commands, 'Navigator TF cache must not issue duplicate stops or cancel healthy controller execution'
                driver.emit_tf = True
            elif scenario in ['localization_short_pause', 'localization_resume', 'pause_timeout', 'paused_status_timeout']:
                driver.phase_override = 'WAIT_LOCALIZATION' if scenario == 'localization_resume' else 'PAUSED_LOCALIZATION'
                if scenario == 'paused_status_timeout':
                    time.sleep(0.3)
                    driver.suppress_status = True
                elif scenario != 'pause_timeout':
                    driver.emit_tf = False
                    time.sleep(0.7 if scenario == 'localization_short_pause' else 1.0)
                    driver.emit_tf = True
                    time.sleep(0.2)
                    driver.phase_override = None
            elif scenario in ['obstacle_resume', 'downstream_stop_resume', 'obstacle_timeout']:
                driver.block = True
                if scenario in ['obstacle_resume', 'downstream_stop_resume']:
                    time.sleep(1.4 if scenario == 'downstream_stop_resume' else 0.6)
                    driver.block = False
            elif scenario == 'cancel':
                result(handle.cancel_goal_async())
            status = result(done).status
            expected = 6 if scenario in ['obstacle_timeout', 'planning_failure', 'control_failure', 'pause_timeout', 'paused_status_timeout'] else 5 if scenario == 'cancel' else 4
            assert status == expected, (scenario, status, expected)
            assert driver.plans - plans == 1, (scenario, 'replanning detected')
            assert len(driver.frozen) - frozen == (0 if scenario == 'planning_failure' else 1)
            if scenario == 'localization_resume':
                assert driver.follows - follows == 2, 'Original path must be redispatched after localization recovery'
            if scenario in ['localization_short_pause', 'navigator_tf_delay']:
                assert driver.follows - follows == 1, 'Short localization pause must retain the existing FollowPath'
            if scenario == 'multi_cusp':
                assert driver.follows - follows == 3, 'Two cusps require three fixed-direction FollowPath calls'
            record = {'scenario': scenario, 'status': status, 'planning_calls': driver.plans - plans,
                      'follow_calls': driver.follows - follows, 'frozen_publications': len(driver.frozen) - frozen}
            report['cases'].append(record)
            print(json.dumps(record), flush=True)
            driver.block = False
            time.sleep(0.4)
        report['passed'] = True
    except Exception as error:
        report['passed'] = False
        report['error'] = str(error)
    finally:
        driver.running = False
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        executor.shutdown()
        thread.join()
        driver.destroy_node()
        rclpy.shutdown()
        log.close()
        (output / 'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
