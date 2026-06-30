# mynavigation2

## 构建规则

在 `nav2_ws/` 工作区中构建，不要在 `navigation2/` 源码仓库内执行 `colcon build`：

```bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

下面按 **ROS 2 Navigation2 / Nav2** 仓库解析。Nav2 是 ROS 2 的导航框架，核心由 **BT 行为树调度 + Planner + Controller + Costmap + Recovery/Behavior + Lifecycle** 组成。官方仓库是 `ros-navigation/navigation2`，ROS Index 上 `humble` 分支近期仍有更新记录。([GitHub][1])

## 1. 总体架构

典型导航链路：

```text
RViz / 用户目标点
        ↓
bt_navigator
        ↓
Behavior Tree
        ↓
Planner Server  →  生成全局路径
        ↓
Controller Server  →  跟踪路径并输出 cmd_vel
        ↓
Robot Base
```

同时还有：

```text
Map / Sensor / TF
        ↓
global_costmap / local_costmap
        ↓
planner / controller / behavior 共用环境代价地图
```

## 2. 核心目录

常见源码目录大致如下：

```text
navigation2/
├── nav2_bringup
├── nav2_bt_navigator
├── nav2_behavior_tree
├── nav2_planner
├── nav2_controller
├── nav2_costmap_2d
├── nav2_behaviors
├── nav2_smoother
├── nav2_amcl
├── nav2_map_server
├── nav2_lifecycle_manager
├── nav2_util
├── nav2_core
└── nav2_msgs
```

## 3. 重点包解析

### `nav2_bringup`

启动入口包。里面主要是 launch 文件、参数文件、默认行为树 XML。官方说明它是一个示例 bringup 系统，实际机器人项目通常要复制并改造成自己的 bringup 包。([GitHub][2])

重点看：

```text
nav2_bringup/launch/navigation_launch.py
nav2_bringup/params/nav2_params.yaml
nav2_bringup/behavior_trees/
```

---

### `nav2_bt_navigator`

导航任务总调度器。

它接收：

```text
NavigateToPose
NavigateThroughPoses
```

然后加载行为树 XML，驱动整个导航流程。

可以理解为：

```text
bt_navigator = 导航任务的大脑 / 调度器
```

---

### `nav2_behavior_tree`

行为树节点实现包。官方文档说明它由 `nav2_bt_navigator` 使用，并基于 BehaviorTree.CPP 执行导航或自治任务行为树。([GitHub][3])

典型 BT 节点包括：

```text
ComputePathToPose
FollowPath
ClearCostmap
Spin
BackUp
Wait
GoalReached
RateController
RecoveryNode
PipelineSequence
```

行为树大致逻辑：

```text
是否有目标？
   ↓
计算路径
   ↓
平滑路径，可选
   ↓
跟踪路径
   ↓
失败则恢复行为
   ↓
重新规划或退出
```

---

### `nav2_planner`

全局路径规划服务器。

它本身不是某一个算法，而是插件管理器，真正算法通过 plugin 加载，例如：

```text
NavFnPlanner
SmacPlanner2D
SmacPlannerHybrid
ThetaStarPlanner
```

核心职责：

```text
输入：起点、目标点、global_costmap
输出：nav_msgs/Path
```

---

### `nav2_controller`

局部控制服务器。

职责是沿着全局路径输出速度：

```text
输入：Path + local_costmap + 当前位姿
输出：geometry_msgs/Twist / cmd_vel
```

常见控制器插件：

```text
DWB Controller
Regulated Pure Pursuit
MPPI Controller
Rotation Shim Controller
```

---

### `nav2_costmap_2d`

代价地图系统，是 Nav2 的环境感知核心。

分为：

```text
global_costmap：给全局规划用
local_costmap：给局部避障和控制用
```

典型 Layer：

```text
StaticLayer       地图层
ObstacleLayer     障碍物层
VoxelLayer        3D 障碍物投影层
InflationLayer    膨胀层
KeepoutLayer      禁行区域
SpeedFilter       限速区域
```

---

### `nav2_behaviors`

恢复行为 / 特殊行为服务器。

例如：

```text
spin
backup
wait
drive_on_heading
assisted_teleop
```

当规划或控制失败时，BT 会调用这些行为进行恢复。

---

### `nav2_lifecycle_manager`

负责管理 Nav2 各节点生命周期。

Nav2 大量节点是 ROS 2 Lifecycle Node，有状态：

```text
unconfigured
inactive
active
finalized
```

启动时一般按顺序 configure、activate：

```text
map_server
amcl
planner_server
controller_server
behavior_server
bt_navigator
```

---

### `nav2_core`

接口定义包。

如果你要写自己的插件，重点看这里：

```text
nav2_core::GlobalPlanner
nav2_core::Controller
nav2_core::GoalChecker
nav2_core::ProgressChecker
nav2_core::Behavior
nav2_core::Smoother
```

它定义“插件必须实现哪些函数”。

---

## 4. 一次导航的源码调用链

以 `NavigateToPose` 为例：

```text
RViz 发送目标点
    ↓
nav2_bt_navigator 接收 action
    ↓
加载 navigate_to_pose_w_replanning_and_recovery.xml
    ↓
ComputePathToPose 节点调用 planner_server
    ↓
planner_server 调用 GlobalPlanner 插件
    ↓
生成 nav_msgs/Path
    ↓
FollowPath 节点调用 controller_server
    ↓
controller_server 调用 Controller 插件
    ↓
输出 cmd_vel
    ↓
底盘运动
```

失败时：

```text
FollowPath 失败
    ↓
RecoveryNode 捕获失败
    ↓
ClearCostmap / Spin / BackUp
    ↓
重新 ComputePathToPose
```

## 5. 推荐阅读顺序

源码不要从头看，建议这样读：

```text
1. nav2_bringup/params/nav2_params.yaml
2. nav2_bringup/behavior_trees/*.xml
3. nav2_bt_navigator
4. nav2_behavior_tree/plugins/action
5. nav2_planner
6. nav2_controller
7. nav2_costmap_2d
8. nav2_core
```

## 6. 最重要的理解

Nav2 不是一个单体算法，而是一个 **插件化导航框架**：

```text
行为树负责流程控制
Planner 负责全局路径
Controller 负责局部控制
Costmap 负责环境建模
Behavior 负责失败恢复
Lifecycle 负责节点状态管理
```

你后面如果要改源码，通常不是直接改 Nav2 主流程，而是写自己的：

```text
GlobalPlanner 插件
Controller 插件
Costmap Layer 插件
BT Node 插件
Behavior 插件
```

最建议先从 `nav2_params.yaml` 和行为树 XML 入手，这两个文件基本能看懂 Nav2 的运行骨架。

[1]: https://github.com/ros-navigation/navigation2?utm_source=chatgpt.com "ros-navigation/navigation2: ROS 2 Navigation Framework ..."
[2]: https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/README.md?utm_source=chatgpt.com "navigation2/nav2_bringup/README.md at main"
[3]: https://github.com/ros-planning/navigation2/blob/main/nav2_behavior_tree/README.md?utm_source=chatgpt.com "navigation2/nav2_behavior_tree/README.md at main"
