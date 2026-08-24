# mynavigation2

本仓库是 `nav2_demo` 项目的 Nav2 源码真源。当前推荐使用
`nav2_regulated_modules` 的统一入口，并在启动时选择互斥运行模式：

```text
nav2_regulated_modules/launch/regulated_modules.launch.py
  remote       人工键盘遥控
  autonomous   无行为树的自主规划、平滑与控制
  fixed_path   直接跟踪上游发布的完整路径
```

`myagv_test_bringup/launch/entry.launch.py` 继续保留为标准 BT Nav2 兼容入口。两个导航入口都面向
实车或板端运行，不启动 AMCL、`map2base_tf`、静态 `odom -> base_link` 或自定义
`laserpub`；定位、机器人 TF、雷达和里程计必须由外部系统提供。

快速启动推荐入口：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=autonomous
```

将 `operation_mode` 改为 `remote` 或 `fixed_path` 即可进入另外两种模式。模式不能在运行中热
切换；必须先停止旧 Launch，再启动新模式。

## 安装依赖

以下命令适用于 Ubuntu 22.04 和 ROS 2 Humble。执行前需已配置 ROS 2 官方 apt 软件源。

安装 Nav2 运行依赖：

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

TEB Local Planner 还依赖 g2o：

```bash
sudo apt install ros-humble-libg2o
```

## 编译命令

所有编译命令都必须在 `nav2_ws` 工作空间执行，不要在 `navigation2` 源码仓库内直接编译。

首次编译或需要重建整个工作空间时：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
export MAKEFLAGS="-j4"
colcon build --symlink-install --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --parallel-workers 1 --packages-select myagv_test_bringup --cmake-args -DBUILD_TESTING=OFF
```

编译 `myagv_test_bringup` 及工作空间内它依赖的包：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to myagv_test_bringup \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

依赖已经编译完成，只重新编译 `myagv_test_bringup` 时：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select myagv_test_bringup \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

只重新编译推荐入口及其仓库内依赖时：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install \
  --packages-up-to nav2_regulated_modules \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
```

编译完成后，在当前终端加载新的 install 空间：

```bash
source install/setup.bash
```

## 1. 标准 BT 兼容入口

本节仅说明保留的 `myagv_test_bringup` 入口。新功能和实车三模式运行请直接阅读第 8 节。

推荐从 install 空间启动：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch myagv_test_bringup entry.launch.py
```

板端无图形界面时关闭 RViz：

```bash
ros2 launch myagv_test_bringup entry.launch.py use_rviz:=False
```

默认关键参数：

```text
map:=myagv_test_bringup/maps/out.yaml
params_file:=myagv_test_bringup/params/nav2_params.yaml
use_sim_time:=False
autostart:=true
use_rviz:=True
use_simulator:=False
robot_name:=odom
```

## 2. 标准 BT 入口实际启动内容

当前 `entry.launch.py` 有效启动的主要节点和 launch：

```text
map_server
lifecycle_manager_localization
navigation_launch.py
rviz_launch.py
controlpub
```

可选启动：

```text
use_simulator:=True 时启动 gzserver
use_simulator:=True 且 headless:=False 时启动 gzclient
```

当前保留为注释、不启动：

```text
laserpub_cmd
map2base_tf_cmd
static_robot_to_base_link_cmd
robot_state_publisher_cmd
joint_state_publisher_cmd
robot_description_publisher.py
```

因此目标机运行该 launch 时不依赖：

```text
xacro
joint_state_publisher
robot_state_publisher
```

RViz 中 `RobotModel` 已关闭，主要通过 TF 坐标系、地图、路径、代价地图和激光数据显示导航状态。


## 3. Nav2 数据流

完整运行链路：

```text
1. 地图
   maps/out.yaml
     -> map_server
     -> /map

2. 外部定位与机器人 TF
   localization system
     -> TF map -> base_link
   或
     -> TF map -> odom -> base_link

3. 激光雷达
   c200 lidar driver
     -> /c200_lidar_node/scan
     -> global_costmap / local_costmap obstacle layer

4. 导航目标
   RViz Nav2 Goal 或上层系统
     -> /navigate_to_pose action
     -> bt_navigator

5. 全局规划
   planner_server
     -> global_costmap + /map + TF
     -> nav_msgs/Path

6. 路径平滑
   smoother_server
     -> smoothed path

7. 局部控制
   controller_server
     -> local_costmap + path + TF
     -> /cmd_vel_nav

8. 速度平滑
   velocity_smoother
     -> /cmd_vel

9. 底盘输出
   controlpub
     -> /control_to_uart
     -> chassis driver
     -> robot motion
```

## 4. Nav2 节点链路

`entry.launch.py` 通过 `navigation_launch.py` 启动导航节点：

```text
controller_server
smoother_server
planner_server
behavior_server
bt_navigator
waypoint_follower
velocity_smoother
lifecycle_manager_navigation
```

职责：

- `bt_navigator`：接收 `NavigateToPose` / `NavigateThroughPoses` action，运行行为树。
- `planner_server`：根据地图、global costmap 和 TF 生成全局路径。
- `smoother_server`：对规划路径做平滑。
- `controller_server`：根据路径、local costmap 和 TF 输出 `/cmd_vel_nav`。
- `velocity_smoother`：把 `/cmd_vel_nav` 平滑为 `/cmd_vel`。
- `behavior_server`：规划或控制失败时执行恢复行为。
- `waypoint_follower`：执行多路点任务。
- `lifecycle_manager_navigation`：管理导航节点生命周期。

## 5. 控制器配置

当前 `myagv_test_bringup/params/nav2_params.yaml` 加载多个控制器：

```text
DWB
RPP
MPPI
GracefulController
RotationShimController
```

默认选择：

```yaml
bt_navigator:
  ros__parameters:
    selected_controller: "DWB"
```

切换控制器时，只改 `selected_controller`，取值必须来自：

```yaml
controller_server:
  ros__parameters:
    controller_plugins:
      - DWB
      - RPP
      - MPPI
      - GracefulController
      - RotationShimController
```

建议：

- `DWB`：当前默认控制器，适合传统采样轨迹和 critic 调试。
- `RPP`：适合实车低速路径跟踪。
- `MPPI`：计算量更大，适合局部轨迹优化实验。
- `RotationShimController`：适合先对齐路径方向再跟踪。
- `GracefulController`：适合验证平滑几何控制。

## 6. 启动后检查

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

检查雷达：

```bash
ros2 topic echo /c200_lidar_node/scan --once
```

检查地图：

```bash
ros2 topic echo /map --once
```

检查速度输出：

```bash
ros2 topic echo /cmd_vel
```

检查生命周期：

```bash
ros2 lifecycle nodes
```

判断标准：

- `map -> base_link` 能持续查到。
- `/c200_lidar_node/scan` 有数据。
- 激光 `frame_id` 能接入 TF 树。
- `/map` 正常发布。
- 导航目标发送后 `/cmd_vel` 有输出。
- Nav2 lifecycle 节点进入 active 状态。

## 7. 终端发送导航目标

发送目标前，先确认 `bt_navigator` 已进入 active 状态，且两个 Action Server 可用：

```bash
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_to_pose
ros2 action info /navigate_through_poses
```

以下坐标取自项目根目录 `test.md` 中的导航测试案例。它们用于说明 Action 命令格式；
在实车地图 `myagv_test_bringup/maps/out.yaml` 上运行前，必须先确认目标点位于可通行区域。

### 7.1 单点 Action 导航

向 `/navigate_to_pose` 发送一个 `PoseStamped` 目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 63.481, y: -12.4484, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  behavior_tree: ''
}" --feedback
```
### 7.2 多点 Action 导航

向 `/navigate_through_poses` 一次发送一组 `PoseStamped` 目标，机器人按数组顺序导航：

```bash
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses "{
  poses: [
    {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 0.570, y: -0.50, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    {
      header: {frame_id: 'map'},
      pose: {
        position: {x: 1.78, y: 0.50, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}
      }
    },
    {
      header: {frame_id: 'map'},
      pose: {
        position: {x: -0.60, y: -1.74, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}
      }
    }
  ],
  behavior_tree: ''
}" --feedback
```

`behavior_tree: ''` 表示使用 `bt_navigator` 对应导航类型的默认行为树；`--feedback`
用于在终端持续显示剩余距离、导航时间和恢复次数等反馈。

## 8. 推荐入口：nav2_regulated_modules 三模式运行

`nav2_regulated_modules` 不使用行为树，通过启动参数 `operation_mode` 在三种互斥模式中选择一种。
模式只能在启动时确定，不支持运行中热切换。切换模式时必须先停止旧 Launch，再启动新模式，避免旧
Action、速度命令或 Topic 发布者残留。

### 8.1 模式对照

| `operation_mode` | 上游输入 | 实际数据流 | 适用场景 |
| --- | --- | --- | --- |
| `remote` | 交互式终端键盘 | 键盘 → `myagv_keyboard_control` → `/control_to_uart` | 人工接管、底盘方向和串口联调 |
| `autonomous` | `/goal_pose`、`NavigateToPose`、`NavigateThroughPoses` | Planner → Smoother → FollowPath → Velocity Smoother → `controlpub` | 自主规划并实时导航 |
| `fixed_path` | `/navigation_service`，类型为 `byd_custom_msgs/action/NavigationService` | 业务分段 → Path 生成／校验 → FollowPath → Velocity Smoother → `controlpub` | 上游提供连续直线段或贝塞尔段 |

三种模式都保证 `/control_to_uart` 只有一个发布源：

- `remote` 只启动 `myagv_keyboard_control`，不启动地图、规划、控制、速度平滑、RViz 和
  `controlpub`。
- `autonomous` 和 `fixed_path` 不启动 `myagv_keyboard_control`，由 `controlpub` 把
  `/cmd_vel` 转换为 `/control_to_uart`。
- `fixed_path` 为保持统一 Lifecycle 节点集合仍会启动 Planner Server 和 Smoother Server，但
  `regulated_navigator` 不会向它们发送规划或路径平滑 Goal。

### 8.2 运行前提与公共环境

`remote` 只需要可交互终端、键盘控制包和底盘通信链，不依赖地图、雷达、里程计或定位 TF。

`autonomous` 和 `fixed_path` 启动前必须确保：

- 外部定位能够持续提供 `map -> base_link` TF。
- 雷达发布 `/c200_lidar_node/scan`，类型为 `sensor_msgs/msg/LaserScan`，其 `frame_id` 能通过
  TF 接入 `base_link`。
- 里程计发布 `/odometry`，类型为 `nav_msgs/msg/Odometry`，供 Controller Server 和
  CLOSED_LOOP Velocity Smoother 使用。
- 底盘控制节点能够接收 `/control_to_uart`。

每个终端先加载环境：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOG_DIR=/tmp/nav2_logs
export SPDLOG_WRAPPER_LOG_DIR=/tmp/nav2_logs
```

`ROS_LOG_DIR` 和 `SPDLOG_WRAPPER_LOG_DIR` 必须指向可写目录。

### 8.3 Launch 参数

统一入口：

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=autonomous
```

常用启动参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `operation_mode` | `autonomous` | 只能取 `remote`、`autonomous`、`fixed_path` |
| `map` | 包内 `maps/out.yaml` | 地图 YAML 绝对路径；只影响导航模式 |
| `params_file` | 包内 `params/regulated_modules.yaml` | Planner、Controller、Smoother、Navigator 和 Costmap 参数 |
| `use_sim_time` | `false` | 是否使用 `/clock` |
| `use_rviz` | `True` | 是否启动 RViz；遥控模式始终不启动 RViz |
| `rviz_config_file` | 包内 `rviz/nav2_default_view.rviz` | RViz 配置文件 |
| `autostart` | `true` | 是否由 Lifecycle Manager 自动激活节点 |
| `use_composition` | `False` | 是否把标准 Nav2 组件加载到已有组件容器 |
| `container_name` | `nav2_regulated_container` | 组合模式使用的外部组件容器名称 |
| `use_respawn` | `False` | 非组合模式下节点异常退出后是否重启 |
| `log_level` | `info` | ROS 日志等级 |
| `namespace` | 空 | 顶层命名空间 |
| `use_namespace` | `False` | 是否启用顶层命名空间 |
| `keyboard_input_device` | 当前 Shell 的 `/dev/pts/*` 或 `/dev/tty` | 遥控模式读取的终端设备 |

`operation_mode` 的命令行值会覆盖 YAML 中 `regulated_navigator.operation_mode`。默认保持
`use_composition:=False`；启用组合模式前必须先准备与 `container_name` 一致的组件容器。

### 8.4 遥控模式

必须从能够接收键盘输入的交互式终端启动：

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=remote
```

Launch 会自动把启动 Shell 的真实终端设备传给键盘节点。自动识别失败时显式指定：

```bash
tty
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=remote \
  keyboard_input_device:=/dev/pts/N
```

统一三模式 Launch 只传入 `input_device` 和 `output_topic`，不会加载
`myagv_keyboard_control/config/keyboard_control.yaml`。因此该入口使用键盘节点内置的速度和平滑
默认值；需要自定义遥控参数时，使用第 10 节的独立 Launch 或 `ros2 run --ros-args -p`。

### 8.5 自主规划模式

启动完整自研自主链：

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=autonomous \
  use_sim_time:=false \
  use_rviz:=True
```

无图形环境：

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=autonomous \
  use_rviz:=False
```

数据流：

```text
/goal_pose／NavigateToPose／NavigateThroughPoses
  -> regulated_navigator
  -> ComputePathToPose／ComputePathThroughPoses
  -> SmoothPath
  -> FollowPath
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel
  -> controlpub
  -> /control_to_uart
```

发送标准单点目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 0.569, y: 0.541, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  behavior_tree: ''
}" --feedback
```

`behavior_tree` 必须为空；该入口不加载外部行为树 XML。RViz 的 `2D Goal Pose` 也可以通过
`/goal_pose` 进入相同的规划、平滑和控制链。

自主链的关键参数位于 `params/regulated_modules.yaml`：

| 参数路径 | 默认值 | 作用 |
| --- | --- | --- |
| `planner_server.selected_planner` | `GridBasedAstar` | Planner Server 最终采用的规划插件；非空时覆盖 Action 中的 `planner_id` |
| `regulated_navigator.planner_id` | `GridBasedAstar` | `regulated_navigator` 写入规划 Action Goal 的插件 ID |
| `regulated_navigator.controller_id` | `DWB` | FollowPath 使用的控制器插件 ID |
| `regulated_navigator.smoother_id` | `simple_smoother` | SmoothPath 使用的平滑器插件 ID |
| `regulated_navigator.use_smoother` | `true` | 是否执行规划后路径平滑 |
| `regulated_navigator.replan_frequency` | `1.0` | 控制期间周期重规划频率，单位 Hz |
| `regulated_navigator.feedback_frequency` | `5.0` | 外层导航 Action 反馈频率，单位 Hz |
| `regulated_navigator.max_recovery_rounds` | `2` | 清理双 Costmap 后重新规划的最大轮数 |
| `velocity_smoother.feedback` | `CLOSED_LOOP` | 使用 `/odometry` 实测速度作为平滑起点 |
| `velocity_smoother.max_velocity` | `[0.26, 0.0, 1.0]` | X、Y、Theta 三轴最大速度 |
| `velocity_smoother.max_accel` | `[2.5, 0.0, 3.2]` | X、Y、Theta 三轴最大加速度 |
| `velocity_smoother.max_decel` | `[-2.5, 0.0, -3.2]` | X、Y、Theta 三轴最大减速度 |

切换全局规划器时，应同时确认 `planner_server.planner_plugins` 已加载目标插件，并让
`planner_server.selected_planner` 与 `regulated_navigator.planner_id` 保持一致。切换控制器时，
目标 ID 必须存在于 `controller_server.controller_plugins`。这些参数由节点配置阶段读取，修改
YAML 后应重新启动 Launch。

当前已加载的规划器 ID 为 `GridBased`、`GridBasedAstar`、`Smac2D`、`SmacHybrid`、
`SmacLattice` 和 `ThetaStar`；控制器 ID 为 `DWB`、`RPP`、`MPPI`、
`GracefulController` 和 `RotationShimController`。

### 8.6 固定路径模式

终端一启动固定路径模式：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=fixed_path \
  use_sim_time:=false \
  use_rviz:=True
```

终端二发送一条直线段，并持续显示整体进度反馈和最终结果：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 action send_goal /navigation_service \
  byd_custom_msgs/action/NavigationService \
  "{task_id: demo_line, navi_segment: [
    {segment_type: 1, node1: {x: 70.1, y: -12.4, z: 0.0}, node2: {x: 63.8, y: -12.1, z: 0.0}, control_pos1: {x: 0.0, y: 0.0, z: 0.0}, control_pos2: {x: 0.0, y: 0.0, z: 0.0}}
  ]}" --feedback
```

#### 三段连续路径示例

以下 Goal 由“直线＋三次贝塞尔曲线＋直线”构成三段连续路径。第一段终点
`(-13.8, -12.0)` 同时是曲线起点，曲线终点 `(17.9, -16.0)` 同时是第三段起点：

```bash
ros2 action send_goal /navigation_service \
  byd_custom_msgs/action/NavigationService \
  "{task_id: demo_three_segments, navi_segment: [
    {
      segment_type: 1,
      segment_name: line_1,
      segment_id: segment_1,
      node1: {x: 9.3, y: -12.0, z: 0.0},
      node2: {x: 14.8, y: -12.0, z: 0.0},
      control_pos1: {x: 0.0, y: 0.0, z: 0.0},
      control_pos2: {x: 0.0, y: 0.0, z: 0.0}
    },
    {
      segment_type: 2,
      segment_name: bezier_1,
      segment_id: segment_2,
      node1: {x: 14.8, y: -12.0, z: 0.0},
      node2: {x: 17.9, y: -16.0, z: 0.0},
      control_pos1: {x: 16.9, y: -12.0, z: 0.0},
      control_pos2: {x: 18.0, y: -13.9, z: 0.0}
    },
    {
      segment_type: 1,
      segment_name: line_2,
      segment_id: segment_3,
      node1: {x: 17.9, y: -16.0, z: 0.0},
      node2: {x: 17.9, y: -20.6, z: 0.0},
      control_pos1: {x: 0.0, y: 0.0, z: 0.0},
      control_pos2: {x: 0.0, y: 0.0, z: 0.0}
    }
  ]}" \
  --feedback
```

其中 `segment_type: 1` 表示直线段，`segment_type: 2` 表示依次经过
`node1/control_pos1/control_pos2/node2` 的三次贝塞尔段。

#### 8.6.1 指定直线段路径点簇

下面给出一条从 `(70.1, -12.4)` 到 `(63.8, -12.1)` 的固定直线段路径，坐标系为 `map`。线段长度约为
`6.307139 m`；按 `0.15 m` 间隔采样并强制包含终点后，共生成 44 个 Pose，最后一个补偿段约为
`0.007139 m`。所有 Pose 使用沿路径前进方向的统一朝向，`yaw=3.094010 rad`，对应四元数约为
`{x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}`。

以下离散 Path 点簇是旧接口的几何说明，不再作为当前 Action 的可执行命令；当前接口只发送分段端点和控制点：

```bash
# 旧离散 Path 示例（仅供几何参考，不能直接发送给 NavigationService）
  "{path: {header: {frame_id: map}, poses: [
    {pose: {position: {x: 70.100000, y: -12.400000, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 69.950169779, y: -12.392865228, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 69.800339559, y: -12.385730455, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 69.650509338, y: -12.378595683, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 69.500679117, y: -12.371460910, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 69.350848897, y: -12.364326138, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 69.201018676, y: -12.357191366, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 69.051188455, y: -12.350056593, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 68.901358235, y: -12.342921821, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 68.751528014, y: -12.335787048, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 68.601697793, y: -12.328652276, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 68.451867573, y: -12.321517503, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 68.302037352, y: -12.314382731, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 68.152207131, y: -12.307247959, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 68.002376911, y: -12.300113186, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 67.852546690, y: -12.292978414, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 67.702716469, y: -12.285843641, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 67.552886249, y: -12.278708869, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 67.403056028, y: -12.271574097, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 67.253225807, y: -12.264439324, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 67.103395587, y: -12.257304552, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 66.953565366, y: -12.250169779, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 66.803735146, y: -12.243035007, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 66.653904925, y: -12.235900235, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 66.504074704, y: -12.228765462, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 66.354244484, y: -12.221630690, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 66.204414263, y: -12.214495917, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 66.054584042, y: -12.207361145, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 65.904753822, y: -12.200226372, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 65.754923601, y: -12.193091600, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 65.605093380, y: -12.185956828, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 65.455263160, y: -12.178822055, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 65.305432939, y: -12.171687283, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 65.155602718, y: -12.164552510, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 65.005772498, y: -12.157417738, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 64.855942277, y: -12.150282966, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 64.706112056, y: -12.143148193, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 64.556281836, y: -12.136013421, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 64.406451615, y: -12.128878648, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 64.256621394, y: -12.121743876, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 64.106791174, y: -12.114609104, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 63.956960953, y: -12.107474331, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 63.807130732, y: -12.100339559, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}},
    {pose: {position: {x: 63.800000, y: -12.100000, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.999717, w: 0.023789}}}
  ]}}" --feedback
```

输入要求：

- Action 默认是 `/navigation_service`，可通过 `regulated_navigator.navigation_service_action` 修改。
- Goal 输入为 `string task_id` 和 `NaviSegment[] navi_segment`；路径生成只读取每段的 `segment_type`、`node1`、`node2`、`control_pos1`、`control_pos2`。
- Feedback 为 `cur_task_id`、空的 `cur_seg_id` 和 `float32 progress`；`progress` 表示总路径完成比例，范围为 `0.0–1.0`。Result 为 `bool finish`。
- `segment_type=1` 为直线段，节点根据 `node1/node2` 自动生成共线控制点；`segment_type=2` 为三次贝塞尔段，依次使用 `node1/control_pos1/control_pos2/node2`。
- 所有坐标按 `global_frame` 解释且必须为有限值；相邻段的前段 `node2` 与后段 `node1` 必须连续，节点按 `fixed_path_step` 近似等距采样并生成切线朝向。
- 上游仍负责路径几何、避障、可行性以及与机器人运动学约束的一致性；插值只增加路径密度，不会把不可行线段变成可行路径。

运行中发送新的 `/navigation_service` Goal 会先校验并生成新 Path，再取消旧 FollowPath、终止旧
外层 Goal，并启动新任务，不需要重启控制器。客户端取消时返回 `CANCELED` 和 `finish=false`；
控制器到达终点时返回 `SUCCEEDED` 和 `finish=true`；取消、抢占或失败时为 `finish=false`。定位恢复时只重发已保存路径，不会回落到
自主规划。该模式会拒绝
`NavigateToPose`、`NavigateThroughPoses` 和 `/goal_pose` 自主目标。

固定路径仍使用以下控制参数：

- `regulated_navigator.controller_id` 和 `goal_checker_id`。
- `controller_server` 对应控制器插件参数。
- `velocity_smoother` 的速度、加减速度和 `/odometry` 闭环参数。

它不使用 `planner_id`、`use_smoother` 或 `replan_frequency` 执行规划。

### 8.7 启动后检查与停止

自主或固定路径模式检查：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /smoother_server
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /regulated_navigator
ros2 param get /regulated_navigator operation_mode
ros2 topic info /control_to_uart --verbose
```

固定路径模式额外检查：

```bash
ros2 action info /navigation_service
ros2 interface show byd_custom_msgs/action/NavigationService
```

遥控模式检查：

```bash
ros2 node list
ros2 topic info /control_to_uart --verbose
```

预期结果：

- 自主和固定路径模式的 Lifecycle 节点均为 `active [3]`。
- `remote` 中 `/control_to_uart` 只有 `myagv_keyboard_control` 发布。
- `autonomous` 和 `fixed_path` 中 `/control_to_uart` 只有 `controlpub` 发布。
- `fixed_path` 中 `/navigation_service` 有一个 `regulated_navigator` Action Server，并且不再存在
  `/fixed_path` Topic 订阅入口。

测试完成后先停止 `regulated_modules.launch.py`，再停止雷达、定位和底盘通信节点。导航模式停止
时，Lifecycle 会先停用 `regulated_navigator`，取消下游 Action 并发布零速度。


## 9. 外部贡献：Fork＋Pull Request

本仓库公开地址：

```text
https://github.com/zpy560/mynavigation2
```

普通外部贡献者不需要本仓库的写权限。推荐使用“Fork 到个人账号、在个人 Fork 开发、向本仓库
`main` 分支提交 Pull Request”的方式贡献代码。

### 9.1 Fork仓库

贡献者登录 GitHub 后打开：

```text
https://github.com/zpy560/mynavigation2/fork
```

选择自己的个人账号并创建 Fork。完成后，贡献者会得到：

```text
https://github.com/CONTRIBUTOR_ACCOUNT/mynavigation2
```

其中 `CONTRIBUTOR_ACCOUNT` 需要替换为贡献者自己的 GitHub 用户名。

### 9.2 克隆个人Fork并添加上游仓库

```bash
git clone https://github.com/CONTRIBUTOR_ACCOUNT/mynavigation2.git
cd mynavigation2

git remote add upstream https://github.com/zpy560/mynavigation2.git
git remote -v
```

远端职责：

| 远端 | 仓库 | 用途 |
| --- | --- | --- |
| `origin` | 贡献者自己的 Fork | 推送贡献者的开发分支 |
| `upstream` | `zpy560/mynavigation2` | 获取本仓库最新代码 |

### 9.3 创建开发分支

不要直接在个人 Fork 的 `main` 分支开发。先创建能够表达改动目的的分支：

```bash
git switch -c fix/map-server-lifecycle
```

其他分支名示例：

```text
feat/keyboard-dev-tty
docs/update-bringup-guide
fix/dual-lidar-tf
```

### 9.4 修改、验证并提交

完成修改后，先检查变更范围和验证结果：

```bash
git status
git diff --check
git diff
```

只暂存本次贡献相关的文件：

```bash
git add PATH_TO_CHANGED_FILE
git commit -m "fix: 修复具体问题并说明影响范围"
```

推荐的提交前缀：

| 前缀 | 用途 |
| --- | --- |
| `feat:` | 新增功能 |
| `fix:` | 修复问题 |
| `docs:` | 更新文档 |
| `refactor:` | 不改变功能的结构调整 |
| `chore:` | 构建、依赖或工程维护 |

### 9.5 推送个人分支

```bash
git push -u origin fix/map-server-lifecycle
```

贡献者只向自己的 `origin` 推送，不需要也不应直接向 `zpy560/mynavigation2` 的 `main` 分支推送。

### 9.6 创建Pull Request

推送后，在 GitHub 页面点击 `Compare & pull request`，并确认目标关系：

```text
base repository: zpy560/mynavigation2
base branch:     main

head repository: CONTRIBUTOR_ACCOUNT/mynavigation2
compare branch:  fix/map-server-lifecycle
```

PR 描述至少应包含：

- 修改了什么。
- 为什么需要修改。
- 影响哪些包、节点、Topic、Action、Service 或参数。
- 使用了哪些验证命令。
- 哪些运行环境尚未验证。

提交后的 PR 会显示在：

```text
https://github.com/zpy560/mynavigation2/pulls
```

### 9.7 根据审查意见更新PR

如果维护者提出修改意见，贡献者继续在同一个开发分支修改并推送即可：

```bash
git add PATH_TO_CHANGED_FILE
git commit -m "fix: 根据审查意见修正具体问题"
git push
```

新的提交会自动追加到现有 PR，不需要重新创建 PR。

### 9.8 同步上游main分支

当 PR 开发期间上游 `main` 有新提交时，可以把上游更新合并到当前开发分支：

```bash
git fetch upstream
git switch fix/map-server-lifecycle
git merge upstream/main
git push
```

如果出现冲突，应在本地解决冲突、重新验证后再推送，不要在不了解影响时覆盖上游文件。

### 9.9 权限边界

- 外部贡献者可以 Fork 公开仓库并提交 PR。
- 外部贡献者默认不能直接推送本仓库，也不能自行合并 PR。
- 仓库维护者负责审查、要求修改、批准、关闭或合并 PR。
- 长期可信任的协作者可以单独授予仓库写权限，但普通外部贡献优先使用 Fork＋PR。
- PR 被合并前，变更不会进入本仓库的 `main` 分支。

## 10. myagv_keyboard_control 键盘控制

`myagv_keyboard_control` 通过交互式终端读取方向键或 `WASD`，以固定周期直接发布
`byd_custom_msgs/msg/ControlRes` 到 `/control_to_uart`。按键只更新目标速度，节点依据独立的线
速度和角速度加减速限制生成连续输出。该节点绕过 `/cmd_vel` 和 `controlpub`，适合人工接管、
底盘速度符号检查和串口控制链联调。

### 10.1 平滑控制行为

| 按键 | 目标动作 | `v` | `w` |
| --- | --- | ---: | ---: |
| `W` 或 `↑` | 平滑加速前进 | `→ +linear_speed` | `0.0` |
| `S` 或 `↓` | 平滑加速后退 | `→ -linear_speed` | `0.0` |
| `A` 或 `←` | 平滑加速原地左转 | `0.0` | `→ +angular_speed` |
| `D` 或 `→` | 平滑加速原地右转 | `0.0` | `→ -angular_speed` |
| `Space` 或 `X` | 按减速度限制平滑停车 | `→ 0.0` | `→ 0.0` |
| `Q` | 立即清零、发布停车指令并退出 | `0.0` | `0.0` |

前进／后退反向、左转／右转反向以及直行／原地转向切换都会先减速到零，再向新目标平滑加速，
不会跨过零点跳变，也不会在直行与原地转向切换期间同时输出明显的线速度和角速度。

终端无法直接报告按键松开事件。方向键超过 `command_timeout` 没有再次输入时，节点把它视为已经
松键，将目标速度置零并按减速度限制平滑停车。`Space`／`X` 用于正常平滑停车；`Q` 用于立即
清零、发布停车指令并退出。

### 10.2 三种启动方式

先加载工作空间：

```bash
cd /home/byd/Documents/zpy_ws/project/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

方式一，推荐使用统一入口进入互斥遥控模式：

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=remote
```

该方式自动关闭自动导航输出链，但使用键盘节点内置参数，不加载键盘 YAML。

方式二，使用键盘包独立 Launch。该入口加载
`myagv_keyboard_control/config/keyboard_control.yaml`：

```bash
ros2 launch myagv_keyboard_control keyboard_control.launch.py
```

方式三，直接运行并用命令行覆盖参数：

```bash
ros2 run myagv_keyboard_control myagv_keyboard_control_node --ros-args \
  -p input_device:=/dev/tty \
  -p publish_rate:=50.0 \
  -p linear_speed:=0.15 \
  -p angular_speed:=0.4 \
  -p linear_accel_limit:=0.3 \
  -p linear_decel_limit:=0.6 \
  -p angular_accel_limit:=0.8 \
  -p angular_decel_limit:=1.6 \
  -p command_timeout:=0.6
```

三种方式都必须在能够接收键盘输入的交互式 Shell 中启动。

### 10.3 参数说明

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `input_device` | `/dev/tty` | 键盘输入终端；Launch 通常覆盖为启动 Shell 的 `/dev/pts/*` |
| `output_topic` | `/control_to_uart` | 最终底盘控制 Topic |
| `publish_rate` | `50.0` | 周期发布频率，单位 Hz |
| `linear_speed` | `0.2` | 前进和后退速度绝对值，单位 m/s |
| `angular_speed` | `0.5` | 左右转角速度绝对值，单位 rad/s |
| `linear_accel_limit` | `0.4` | 线速度加速限制，单位 m/s²，必须大于零 |
| `linear_decel_limit` | `0.8` | 线速度减速限制绝对值，单位 m/s²，必须大于零 |
| `angular_accel_limit` | `1.0` | 角速度加速限制，单位 rad/s²，必须大于零 |
| `angular_decel_limit` | `2.0` | 角速度减速限制绝对值，单位 rad/s²，必须大于零 |
| `command_timeout` | `0.5` | 最后一次方向输入后的松键判定超时，单位 s；`0.0` 表示关闭超时停车 |

参数约束：

- `publish_rate` 和四个加减速度限制必须大于零。
- 目标线速度、目标角速度和 `command_timeout` 必须为有限非负数。
- 参数在节点启动时读取，当前没有运行期动态参数回调；不要依赖启动后的
  `ros2 param set` 改变实际控制行为。
- 按住方向键时，终端依靠系统键盘重复事件持续刷新命令。`command_timeout` 应大于实际按键重复
  间隔，否则持续按键期间也可能反复进入减速。

### 10.4 参数调节方法

默认 `50 Hz` 下：

```text
线速度加速时间 = linear_speed / linear_accel_limit = 0.2 / 0.4 = 0.5 s
线速度停车时间 = linear_speed / linear_decel_limit = 0.2 / 0.8 = 0.25 s
角速度加速时间 = angular_speed / angular_accel_limit = 0.5 / 1.0 = 0.5 s
角速度停车时间 = angular_speed / angular_decel_limit = 0.5 / 2.0 = 0.25 s
```

调参原则：

- 起步冲击大：降低 `linear_accel_limit` 和 `angular_accel_limit`。
- 松键停车过猛：降低 `linear_decel_limit` 和 `angular_decel_limit`。
- 停车距离过长：适度提高减速度限制，但必须结合底盘负载和轮地附着验证。
- 松键后停车太晚：降低 `command_timeout`，同时保证它仍大于键盘重复间隔。
- 控制输出不连续：先检查 `publish_rate` 是否稳定，再检查终端输入是否持续刷新。

长期参数写入：

```yaml
# myagv_keyboard_control/config/keyboard_control.yaml
myagv_keyboard_control:
  ros__parameters:
    publish_rate: 50.0
    linear_speed: 0.15
    angular_speed: 0.4
    linear_accel_limit: 0.3
    linear_decel_limit: 0.6
    angular_accel_limit: 0.8
    angular_decel_limit: 1.6
    command_timeout: 0.6
```

该 YAML 只由键盘包独立 Launch 加载。统一三模式入口的遥控分支目前使用节点内置值。

### 10.5 检查输出

另开一个已经加载工作空间环境的终端：

```bash
ros2 topic echo /control_to_uart
ros2 topic hz /control_to_uart
ros2 topic info /control_to_uart --verbose
```

默认输出接口：

```text
Topic: /control_to_uart
Type:  byd_custom_msgs/msg/ControlRes
Rate:  50 Hz
```

`v_lift` 和 `w_rotation` 应始终为 `0.0`。切换方向或松键后，观察 `v`、`w` 是否按斜坡逐步过零，
而不是一步跳变。

### 10.6 实车安全要求

运行前检查 `/control_to_uart` 的发布者，必须确保只有当前控制链：

```bash
ros2 topic info /control_to_uart --verbose
```

不要同时运行 `controlpub` 和独立键盘节点。两者都会发布 `/control_to_uart`，同时运行会造成底盘
指令竞争。使用 `operation_mode:=remote` 时，统一 Launch 已通过互斥分支避免这个问题。

首次测试应架空驱动轮或断开动力执行机构，先检查前进、后退和左右转的速度符号，再连接真实底盘。
终端失去焦点、SSH 中断或节点异常退出后，不能只依赖软件自动停车；底盘控制器还应具备独立的通信
超时停车保护。

结束控制时按 `Q`，节点会先发布停车指令再退出。不要直接关闭终端代替正常停车流程。

### 10.7 终端设备排障

Launch 已支持读取启动 Shell 的真实 `/dev/pts/*`，不再受旧版“子进程标准输入不是终端”的限制。
如果自动识别失败，先查询当前终端：

```bash
tty
```

再显式传入设备：

```bash
ros2 launch myagv_keyboard_control keyboard_control.launch.py \
  input_device:=/dev/pts/N
```

或在统一三模式入口中使用：

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=remote \
  keyboard_input_device:=/dev/pts/N
```

设备必须属于当前交互式终端并具有读取权限。节点退出时会恢复原终端属性。

## 中文翻译

# mynavigation2 项目说明

本项目是当前工作区中的 Nav2 源码和机器人应用集合。README 前面的章节说明标准 BT 导航入口、导航数据流、节点链路、控制器选择、启动后检查以及通过 NavigateToPose 和 NavigateThroughPoses Action 发送目标。

nav2_regulated_modules 提供 remote、autonomous 和 fixed_path 三种互斥运行模式。remote 只启动键盘遥控并向底盘输出速度；autonomous 运行标准规划、控制、平滑和速度限制链；fixed_path 通过 NavigationService Action 接收直线或贝塞尔分段，并复用 Nav2 Controller Server 执行。每种模式的 Launch 参数、输入 Topic、输出 Topic、前提条件和停止方式在原文对应章节中列出。

贡献流程包括 Fork、克隆个人仓库、添加上游远端、创建开发分支、修改和验证、提交、推送以及创建 Pull Request。导航系统的安全边界是保持单一最终速度发布者、切换时先停止旧任务、校验 Path 和 TF，并在设备退出时恢复终端属性。原文代码、命令、参数和接口名称保持不变。
