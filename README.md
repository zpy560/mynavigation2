# mynavigation2

本仓库是 nav2_demo 项目当前开发的 Nav2 源码，直接位于 `nav2_ws/src/navigation2/`，没有另一份工作空间源码副本。基于 ROS 2 Humble，包含标准 Nav2、无 BT 四模式规控、自定义底盘接口和仿真场景。

## 入口选择

| 用途 | 入口 | 定位与环境 |
|---|---|---|
| 推荐规控：remote／autonomous／fixed_path／autonomous_fixed | [nav2_regulated_modules](nav2_regulated_modules/README.md) | 外部定位、传感器、轮速和底盘通信；不启动仿真 |
| 当前仓库场景仿真 | [myworld_bringup](myworld_bringup/README.md) | Ignition、桥接和四模式装配；导航定位须外部启动 |
| TurtleBot3 仿真 | [mytb3_bringup](mytb3_bringup/README.md) | 独立仿真时钟与场景配置 |
| 另一套 myworld 仿真 | [myworld_bringup_sim](myworld_bringup_sim/README.md) | 保留兼容入口，不等同当前四模式装配 |
| 键盘直接控制底盘 | [myagv_keyboard_control](myagv_keyboard_control/README.md) | 交互终端；直接输出 ControlRes |

模式启动后不能热切换。导航模式要有新鲜的定位 TF 与速度反馈；启动进程、发现 Action 或加载配置均不等于车辆已具备运行条件。

## 安装依赖

以下命令适用于 Ubuntu 22.04／ROS 2 Humble，执行前需已安装 ROS 2 Humble 并配置 ROS 2 官方 apt 软件源。

安装 Nav2 运行依赖：

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

本仓库的 TEB Local Planner 还依赖 g2o；全量构建包含该包时先安装：

```bash
sudo apt install ros-humble-libg2o
```

碰撞监视器的激光／点云半径离群点过滤还使用 PCL 的 `common`、`filters` 模块，依赖已在 [nav2_collision_monitor/package.xml](nav2_collision_monitor/package.xml) 和 [CMakeLists.txt](nav2_collision_monitor/CMakeLists.txt) 声明；按下节通过 rosdep 安装对应依赖。

源码其余依赖按下节的 `rosdep install` 命令安装，依赖声明见各包的 `package.xml`。仿真另需 Ignition Gazebo 与 ros_gz 组件，入口说明见 [myworld_bringup](myworld_bringup/README.md) 和 [TurtleBot3 仿真](navigation2_ignition_gazebo_turtlebot3/README.md)。完成源码构建后，按下节加载本工作空间的 `install/local_setup.bash`。

## 目录与构建

源码含 54 个 ROS 包；53 个由当前 colcon 发现，nav2_system_tests 因 COLCON_IGNORE 默认跳过。嵌套 DWB 子包和 turtlebot3 按各自 package.xml 的名称构建，不能用仓库目录名代替包名。

命令从外层 nav2_demo 项目根目录开始；已在 nav2_ws 时跳过 cd。不在本源码仓库或包目录中运行 colcon，也不同时叠加 learningnav2_ws 环境。

```bash
cd nav2_ws
source /opt/ros/humble/setup.bash
# 首次准备依赖；需要已配置 ROS 2 / rosdep，实际安装由使用者执行。
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

仅构建规控及其工作空间依赖：

```bash
# 在 nav2_ws，先加载 /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to nav2_regulated_modules \
  --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
```

依赖基线为 Ubuntu 22.04／ROS 2 Humble；仿真另需 Ignition Gazebo 与 ros_gz 组件，TEB 另依赖 g2o。具体依赖看各包 manifest 和构建声明。本轮未安装依赖；历史全量构建关闭了测试，不能据此声称所有测试通过。构建临时文件和 ROS 日志可通过 TMPDIR、ROS_LOG_DIR 指向可写数据盘，避免系统盘不足。

## 推荐运行与数据流

每个终端加载 ROS 2 Humble 与同一个 `nav2_ws/install/local_setup.bash`，不要叠加学习工作空间。以下入口互斥，只运行当前需要的一条；已有后台时不重复启动。实车驱动和外部定位不由直接入口启动。导航三模式先设置可读地图文件，remote 不需要地图规划：

```bash
MAP_FILE='/userdata/map/pbstream/out.yaml'
test -r "$MAP_FILE"
```

### 四种模式的实车入口

```bash
# fixed_path：分段固定路径，使用 NavigationService。
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=fixed_path use_rviz:=false use_sim_time:=false \
  map:="${MAP_FILE:?先设置实际地图路径}"
```

```bash
# remote：交互终端键盘输出 ControlRes，不启动导航链。
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=remote use_rviz:=false use_sim_time:=false
```

```bash
# autonomous_fixed：单点规划一次，冻结后按固定路径执行。
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=autonomous_fixed use_rviz:=false use_sim_time:=false \
  map:="${MAP_FILE:?先设置实际地图路径}"
```

```bash
# autonomous：自主单点／多点导航，可重新规划。
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=autonomous use_rviz:=false use_sim_time:=false \
  map:="${MAP_FILE:?先设置实际地图路径}"
```

三个导航模式需要新鲜 `map→base_link`、`/scan`、`/motion_state` 和底盘执行链。Cartographer 在其实际部署工作空间中独立启动，地图／pbstream 与导航地图一致；本仓库规控入口不启动 AMCL。完整前提与模式参数见[实车操作说明](nav2_regulated_modules/README.md#实车四种模式的启动)。

### 当前场景的四模式仿真

先加载同一工作空间，导航模式沿用上面的 `MAP_FILE`。按模式只选一条；`use_rviz=false` 是以下示例的显式选择，不改变场景默认值：

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=autonomous use_gazebo:=true headless:=true \
  use_rviz:=false use_sim_time:=false map:="${MAP_FILE:?先设置实际地图路径}"
```

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=fixed_path use_gazebo:=true headless:=true \
  use_rviz:=false use_sim_time:=false map:="${MAP_FILE:?先设置实际地图路径}"
```

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=autonomous_fixed use_gazebo:=true headless:=true \
  use_rviz:=false use_sim_time:=false map:="${MAP_FILE:?先设置实际地图路径}"
```

```bash
ros2 launch myworld_bringup entry.launch.py \
  operation_mode:=remote use_gazebo:=true headless:=true \
  use_rviz:=false use_sim_time:=false
```

仿真导航模式由 entry 同时装配 Gazebo、桥接和规控，不能再重复启动直接规控入口。外部 Cartographer 仍提供定位 TF；Map Server 的 `/map` 不产生定位 TF。默认 `headless=true` 只关闭 Gazebo 图形窗口，导航三模式的内置 RViz 默认仍为 true，本文显式关闭以便独立打开。remote 不启动内置 RViz 或键盘，只接收仿真遥控 ChassisControl，详见[场景四模式说明](myworld_bringup/README.md#四种模式的仿真启动)。

| 模式 | 直接规控默认选择 | 场景入口默认选择 | 任务接口 |
|---|---|---|---|
| remote | 键盘→`/control_to_uart`（ControlRes） | `/downstream/chassis_control`（ChassisControl）→网关→`/cmd_vel` | 无导航 Action，两种输入未自动互接 |
| autonomous | SmacHybrid／DUBIN＋RPP | SmacHybrid／REEDS_SHEPP＋专用 MPPI | 单点、多点导航 Action 或 `/goal_pose` |
| fixed_path | 业务 Path＋FixedPathController | 同类执行链，使用场景参数覆盖 | NavigationService |
| autonomous_fixed | 项目 Hybrid／REEDS_SHEPP 一次搜索＋冻结执行 | 同类执行链，使用场景参数及模式重写 | 单点导航 Action 或 `/goal_pose`，拒绝多点 |

直接 autonomous 的 `autonomous_planner_id` 默认 SmacHybrid，会覆盖共享 YAML 中旧的 GridBasedAstar 选择；显式传空才沿用 YAML。场景 autonomous 按 `myworld2.yaml` 选择规划器，并默认启用专用 MPPI；autonomous_fixed 独立覆盖为项目 Hybrid 包装、REEDS_SHEPP 和 10 m 距离查表。配置来自 YAML、Launch 重写和显式实参，不能只看 YAML 字面值。autonomous 控制／平滑默认 50 Hz，fixed_path 与 autonomous_fixed 为 100 Hz；这些不是实测频率。

显式 `use_gazebo:=false` 会关闭场景的仿真、桥接和速度转换，要求外部提供对应输入，但仍保留场景参数与模型，不等于切回实车共享 YAML。remote 在这一组合下也不启动遥控网关或键盘，实车遥控应选择直接入口。

### Action 与独立 RViz2 的选择

三个导航模式默认系统时间。完整命令见[Action 下发与检查](nav2_regulated_modules/README.md#action-下发与结果检查)和[独立 RViz2](nav2_regulated_modules/README.md#独立打开-rviz2)；场景专用显示配置见[场景 RViz2](myworld_bringup/README.md#仿真-rviz2-与时间设置)。各终端加载同一环境并使用相同 ROS_DOMAIN_ID／DDS 设置。

独立 RViz 的模式配置可按下例选择，先将 `MODE` 设置为 `autonomous`、`fixed_path` 或 `autonomous_fixed`；remote 只作可视化观察时使用 `nav2_default_view`：

```bash
: "${MODE:?先设置对应的RViz配置名}"
rviz2 -d "$(ros2 pkg prefix --share nav2_regulated_modules)/rviz/$MODE.rviz" \
  --ros-args -p use_sim_time:=false
```

只有整个定位／导航／桥接链切为仿真时间且有 `/clock` 时，才同步将独立 RViz 参数改为 true。autonomous 与 autonomous_fixed 可用 `Nav2 Goal` 工具拖动单点目标；fixed_path 必须通过 NavigationService 发送业务路径，remote 不下发导航目标。内置 RViz 的进程联动与独立进程不同，退出独立窗口不等于取消 Action。

```text
autonomous: 导航目标 → regulated_navigator → Planner → 可选 Smoother → FollowPath
fixed_path: NavigationService(NaviSegment[]) → 方向化 Path → FollowPath
autonomous_fixed: 单点目标 → 一次 Hybrid 规划 → 冻结／分段 → FollowPath
FollowPath → Controller Server → /cmd_vel_nav → Velocity Smoother
  → /cmd_vel → controlpub → /control_to_uart
```

启用 Collision Monitor 后，平滑器先发 `/cmd_vel_collision_in`，监控过滤后发 `/cmd_vel`。标准 BT 入口由 bt_navigator 编排，是否调用 SmoothPath 取决于 BT。路径平滑与速度平滑是不同功能。

| 接口 | 类型／含义 |
|---|---|
| `/navigation_service` | `byd_custom_msgs/action/NavigationService`；仅 fixed_path 接受任务 |
| `/navigate_to_pose` | NavigateToPose；autonomous 与 autonomous_fixed 接受 |
| `/navigate_through_poses` | NavigateThroughPoses；四模式入口中仅 autonomous 接受 |
| `/follow_path` | `nav2_msgs/action/FollowPath`；Controller Server |
| `/cmd_vel_nav`、`/cmd_vel` | Twist；控制输出与平滑／过滤后速度 |
| `/odometry` | `nav_msgs/msg/Odometry`；场景桥接的原始仿真反馈，是 MotionState 转换输入 |
| `/motion_state` | `byd_custom_msgs/msg/MotionState`；当前规控／场景默认供控制器、平滑器、导航器和底盘闭环使用 |
| `/downstream/chassis_control` | ChassisControl；底盘指令支路，具体消费者随模式变化 |
| `/control_to_uart` | ControlRes；ROS 底盘输出，不代表本仓库实现 UART 驱动 |

以上为无命名空间入口约定，参数和 remapping 可改变实际名称。定位需提供 map→base_link 或完整等效 TF 链；合成 laserpub／map2base_tf 只用于隔离测试，当前生产入口不自动启动。

反馈类型要与消费者匹配：控制器、平滑器、导航器和统一诊断按话题末级名称是否为 `motion_state` 选择 MotionState／Odometry；MotionState 速度取 `v_car/w_car` 并保留源 Header。实际控制反馈以 `controller_server.odom_topic` 为准，平滑反馈以 `velocity_smoother.odom_topic` 为准，导航器看 `velocity_odom_topic`。当前两套 YAML 已显式配置为 `/motion_state`；自定义文件省略导航器字段时，其 C++ 回退仍为 `/odometry`，不能把不同层默认值混为一谈。

## 底盘 ChassisControl 命令行发布

话题为 `/downstream/chassis_control`，类型为 [`byd_custom_msgs/msg/ChassisControl`](byd_custom_msgs/msg/ChassisControl.msg)。在新终端加载同一工作空间，并与接收端使用相同的 ROS_DOMAIN_ID／DDS 设置：

```bash
source /opt/ros/humble/setup.bash
cd /mnt/nvme/codex/nav2_demo/nav2_ws
source install/local_setup.bash
ros2 topic info /downstream/chassis_control --verbose
```

先确认实际消费者：场景仿真的 remote 模式由遥控网关接收；三个导航模式的 `regulated_navigator` 另有该订阅支路，需要节点 active 和新鲜 `/motion_state`。直接规控入口的 remote 是键盘直控，不自动接收此话题。发布 ChassisControl 不会自动取消导航任务或切换模式，遥控运动前先结束当前导航，并确认底盘输出没有发布者竞争。

| `op` | 操作 | 使用的速度字段 |
|---|---|---|
| `0`（OP_FORWARD） | 前进 | `linear_velocity`，单位 m/s |
| `1`（OP_BACKWARD） | 后退 | `linear_velocity`，单位 m/s |
| `2`（OP_TURN_LEFT） | 原地左转 | `angular_velocity`，单位 rad/s |
| `3`（OP_TURN_RIGHT） | 原地右转 | `angular_velocity`，单位 rad/s |

速度和 `acceleration` 均填写有限、非负幅值，正负方向由 `op` 决定；前进／后退会将角速度置零，左转／右转会将线速度置零。`acceleration` 是对应线／角轴的加速度请求，填 0 使用接收端默认值，实际输出还受限速及 S 曲线约束。下面每次只运行一条运动命令，换方向前先 `Ctrl+C` 停止原发布进程。

```bash
# 前进：0.1 m/s，50 Hz 持续发送。
ros2 topic pub --rate 50 --qos-reliability reliable --qos-durability volatile \
  /downstream/chassis_control byd_custom_msgs/msg/ChassisControl \
  '{header: {frame_id: base_link}, op: 0, linear_velocity: 0.1, angular_velocity: 0.0, acceleration: 0.3}'
```

```bash
# 后退：幅值仍为正数，op=1 决定倒车。
ros2 topic pub --rate 50 --qos-reliability reliable --qos-durability volatile \
  /downstream/chassis_control byd_custom_msgs/msg/ChassisControl \
  '{header: {frame_id: base_link}, op: 1, linear_velocity: 0.1, angular_velocity: 0.0, acceleration: 0.3}'
```

```bash
# 原地左转：0.2 rad/s。
ros2 topic pub --rate 50 --qos-reliability reliable --qos-durability volatile \
  /downstream/chassis_control byd_custom_msgs/msg/ChassisControl \
  '{header: {frame_id: base_link}, op: 2, linear_velocity: 0.0, angular_velocity: 0.2, acceleration: 0.3}'
```

```bash
# 原地右转：幅值仍为正数，op=3 决定右转。
ros2 topic pub --rate 50 --qos-reliability reliable --qos-durability volatile \
  /downstream/chassis_control byd_custom_msgs/msg/ChassisControl \
  '{header: {frame_id: base_link}, op: 3, linear_velocity: 0.0, angular_velocity: 0.2, acceleration: 0.3}'
```

停止原运动发布进程后，用下面命令以 50 Hz 连续发送 20 帧零目标，然后观察 `/motion_state` 的 `v_car/w_car` 确认停稳；没有单独的 STOP 操作码：

```bash
ros2 topic pub --rate 50 --times 20 --qos-reliability reliable --qos-durability volatile \
  /downstream/chassis_control byd_custom_msgs/msg/ChassisControl \
  '{header: {frame_id: base_link}, op: 0, linear_velocity: 0.0, angular_velocity: 0.0, acceleration: 0.3}'
```

只需发送一帧零速消息以检查消息接收或触发速度诊断时，可用：

```bash
ros2 topic pub --once --qos-reliability reliable --qos-durability volatile \
  /downstream/chassis_control byd_custom_msgs/msg/ChassisControl \
  '{header: {frame_id: base_link}, op: 0, linear_velocity: 0.0, angular_velocity: 0.0, acceleration: 0.0}'
```

`ros2 topic pub` 默认只有 1 Hz，而导航器底盘支路默认 `chassis_command_timeout=0.15` s、仿真遥控网关默认 `command_timeout=0.5` s，持续运动应显式使用上面的 50 Hz。超时会触发接收端停车处理；`Ctrl+C` 只结束发布进程，不会自动补发零速。示例 Header 仅设置 `frame_id`，时间戳使用消息默认零值；当前两个消费者按本地接收时间计算命令超时。速度诊断收到首帧 ChassisControl 后开始记录，也不代表底盘已经执行或停稳。

## 自主路径 Action 命令行测试

后台已启动 Cartographer 定位和规控时，在新终端加载同一工作空间即可，不要重复启动导航栈。单点命令适用于仿真／实车的 autonomous 与 autonomous_fixed，多点仅适用于 autonomous；两个自主模式均拒绝非空行为树，fixed_path 和 remote 不接受这些自主目标。若启用了命名空间，给 Action、节点和服务名加上对应前缀。

```bash
source /opt/ros/humble/setup.bash
cd /mnt/nvme/codex/nav2_demo/nav2_ws
source install/local_setup.bash

ros2 param get /regulated_navigator operation_mode
ros2 lifecycle get /regulated_navigator
ros2 lifecycle get /controller_server
ros2 action list -t
```

先确认 `operation_mode` 与所选任务匹配、两个节点均为 `active`，以及 Action 类型正确；发现列表中存在多点 Action 并不代表 autonomous_fixed 会接受它。发送前确认定位 TF、目标通行性和实际轮速。下面坐标变量必须按当前地图设置，单点与多点分别执行；新目标可能抢占旧任务。

### 单点到达并停车

在同一终端设置 `GOAL_X`、`GOAL_Y` 为当前地图的目标坐标，单位米；终点航向为 0°（车头沿地图 +X 方向）。`--feedback` 持续打印反馈；自主 Action 不包含固定路径的 `navi_segment`、`motion_direction` 或 `max_speed` 字段，速度由控制器及平滑器配置决定。两个自主模式的 `behavior_tree` 均必须留空。

```bash
: "${GOAL_X:?先设置地图目标X}" "${GOAL_Y:?先设置地图目标Y}"
ros2 action send_goal --feedback /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, behavior_tree: ''}"
```

修改终点位置时改 `position.x/y`；平面航向通过四元数 `orientation.z/w` 指定，`x/y` 保持 0。例如 90° 使用 `z: 0.7071067811865476, w: 0.7071067811865476`，180° 使用 `z: 1.0, w: 0.0`；不要把角度直接填入 `orientation.z`。

### 按顺序经过多个目标

仅 autonomous 接受此请求。先设置 `VIA_X`、`VIA_Y` 和 `GOAL_X`、`GOAL_Y`；每个元素都需要坐标系、位置和有效四元数。示例中间点 yaw=0，终点 yaw=90°；这不是要求每个中间点都停车等待的站点任务。

```bash
: "${VIA_X:?先设置中间点X}" "${VIA_Y:?先设置中间点Y}"
: "${GOAL_X:?先设置地图目标X}" "${GOAL_Y:?先设置地图目标Y}"
ros2 action send_goal --feedback /navigate_through_poses \
  nav2_msgs/action/NavigateThroughPoses \
  "{poses: [{header: {frame_id: map}, pose: {position: {x: $VIA_X, y: $VIA_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7071067811865476, w: 0.7071067811865476}}}], behavior_tree: ''}"
```

消息字段见 [NavigateToPose.action](nav2_msgs/action/NavigateToPose.action) 和 [NavigateThroughPoses.action](nav2_msgs/action/NavigateThroughPoses.action)。

### 结果判断与取消任务

- `Goal accepted with ID: ...` 仅表示目标被接收，保存该 UUID 以便精确取消。`Goal finished with status: SUCCEEDED` 才表示任务成功；`ABORTED` 表示失败，`CANCELED` 表示取消。
- 反馈包括 `current_pose`、`distance_remaining`、`navigation_time` 和 `number_of_recoveries`；多点任务另有 `number_of_poses_remaining`。自主剩余距离取下游路径估计与当前请求目标直线距离的较大值，避免最近路径点已到末点时提前归零；这仍不是独立物理真值。规划或换路径后首次控制反馈尚未到达时，使用目标直线距离作为下界。
- `estimated_time_remaining` 包含平移、终点转向及默认 0.5 秒停稳的估计，原地转向或低速时不直接置零；估计值不参与成功判定。Duration 要同时看 `sec` 和 `nanosec`，`nanosec: 0` 也可以表示合法的整秒。
- 自主 `number_of_recoveries` 为控制恢复与定位丢失后恢复执行次数之和，受字段范围限制时饱和显示；控制恢复的次数上限仍单独生效。
- 默认场景的最终位置／航向／速度需满足配置阈值并连续稳定 0.5 秒；Action 成功与独立实车精度验收分开评价。失败原因可查看 `/tmp/nav2_logs/nav2_regulated_modules/regulated_navigator.log`，若设置了 `log_dir` 则使用对应目录。
- 位置正好重合时 `distance_remaining` 可以为 0，而航向／轮速仍未达标。此时查看 `controller_server.log` 中的 `Autonomous arrival waiting=...`：`POSITION` 为位置未达标，`HEADING` 为航向未达标，`VELOCITY` 为轮速未达标，`SETTLING` 为等待连续稳定窗口；日志同时给出误差、阈值和稳定时长。成功仍由下游 FollowPath 的成功结果触发，不由距离字段单独触发。
- 本机已核对的 Humble `ros2 action send_goal` 在目标被接收后支持原终端 `Ctrl+C` 请求取消该目标。等待取消结果，并确认 `/motion_state` 的 `v_car/w_car` 停稳；关闭终端或客户端断开不能作为停车确认。

也可在另一终端按 UUID 取消单点任务。将 `GOAL_UUID` 替换成发送命令打印的 32 位目标 ID；转换成功后才发送取消请求，时间戳保持零，仅匹配该 UUID。

```bash
GOAL_UUID='替换为本轮目标的32位UUID'
GOAL_UUID_BYTES=$(python3 -c 'import sys, uuid; u = uuid.UUID(sys.argv[1]); assert u.int != 0, "目标 UUID 不能全为零"; print(list(u.bytes))' "$GOAL_UUID") && \
ros2 service call /navigate_to_pose/_action/cancel_goal \
  action_msgs/srv/CancelGoal \
  "{goal_info: {goal_id: {uuid: $GOAL_UUID_BYTES}, stamp: {sec: 0, nanosec: 0}}}"
```

取消多点任务时，把上述服务名替换为 `/navigate_through_poses/_action/cancel_goal`，UUID 使用该多点任务自己的 ID。`return_code=0` 且 `goals_canceling` 包含目标 UUID 表示取消请求已接受，仍需等待任务结束和轮速停稳。

## 固定路径、collision 与测试

NavigationService 支持直线／三次贝塞尔；各段同向，方向 1 前进、2 倒车，max_speed 始终为正。任务限速取各段最小值，再受 fixed_path_max_speed=1.5 限制。起步航向和速度保护可能先转向或低速跟踪；Action 成功与独立速度／定位验收分别记录。

固定路径直接使用业务分段生成的 Path，不调用全局规划或路径平滑 Action，也不自动搜索绕障路线。`motion_direction=2` 时按路径切线反向编码车头姿态，并以负线速度执行；`max_load_speed`、`dwell_time` 当前未在这条固定路径执行链中实现对应的限速／停留调度，不能仅凭消息字段存在就认为已生效。

当前直接规控参数的起步上限为 `start_offset_speed_limit=0.30 m/s`，仍受更低任务限速约束。保护距离按沿路径投影进度计算：至少从首次有效投影位置再前进 `start_speed_limit_distance=0.50 m`，并且横向／运动方向航向误差连续 10 个周期达标后才解除；这不是车辆到起点的直线距离。起步保护在同一路径解除后不重复开启。

终点参数与成功条件按当前源码分开理解：

- 直接规控默认 `goal_stop_entry_tolerance=0.008 m`；进入停车入口或满足末段越过终点平面条件后，控制器锁存零速并停止继续转向。位置精度仍按 `xy_goal_tolerance=0.01 m` 检查，实际轮速由下游执行链决定。
- `terminal_convergence_enabled=true` 时，优先要求当前位置精度、定位／速度反馈时效及停稳条件连续通过 1 秒复核；已锁停且新鲜反馈持续停稳 2 秒、定位仍新鲜时，也可经兜底返回成功并记录 WARN。因此必须同时检查误差、超限标志和复核日志。
- 直接规控默认终点减速度 `0.20 m/s²`、末段接近速度 `0.015 m/s`，动态制动裕量为低速 `0.010 m`／高速 `0.050 m`，过渡速度 `0.50 m/s`。仿真包装有独立参数与覆盖逻辑，不能把两套配置的调参结果直接混用。
- 终点基础前视为 `goal_terminal_lookahead_dist=0.20 m`，参考速度 `0.75 m/s`，速度增益 `0.10 s`，距离限制为 `0.15～0.25 m`；末端收敛开启时使用实测速度幅值计算。正常前视到末段前视的混合区间为距终点 `0.50～0.10 m`。
- 当前 `selectCarrot()` 仍从最近离散路径点累计长度选点，路径不足前视距离时沿终点切线外推虚拟跟踪点。外推距离为“前视距离减去最近路径点到终点的剩余弧长，再取非负值”；它随位置与参数变化，不是固定延长 15 厘米，也不改变业务终点。连续投影／段内插值的优化方案尚未实现。

实现入口见 [固定路径控制器](nav2_regulated_modules/src/fixed_path_controller.cpp) 、[末端前视混合](nav2_regulated_modules/include/nav2_regulated_modules/detail/terminal_lookahead.hpp) 和 [Controller Server 到点复核](nav2_controller/src/controller_server.cpp) 。

外部 collision 门控在 fixed_path 默认启用，自主／遥控不启用。状态 4／3／2 分别为任务速度上限的 100%／65%／35%，1／0 停止；首次从未收到消息按 4，收到后非法值或断流达到 0.5 秒停车。恢复 4 只解除限制，不创建任务或恢复已取消任务。

Collision Monitor 与上述门控独立，直接规控和当前 myworld 入口 `use_collision_monitor` 均默认 false；区域可视化不产生碰撞状态。Launch 的 enable_collision_state 默认 true 与独立 C++ 节点 collision_state_enabled 的回退 false 属于不同配置层。

- [规控完整说明](nav2_regulated_modules/README.md)：模式、反馈、默认配置与门控状态表。
- [50 Hz collision Action 例子](nav2_regulated_modules/tools/COLLISION_TEST.md)：周期 4→3→2→1→0、1.5 m/s、往返倒车及结果分类。
- [测试工具](nav2_regulated_modules/tools/README.md)：现有环境观察与隔离仿真实验的区别。
- [分开验收](nav2_regulated_modules/doc/action_only_acceptance.md)：速度停稳、定位覆盖与精度独立评价。
- [固定路径架构](nav2_regulated_modules/doc/fixed_path_architecture.md)：既有模块职责与流程；其中旧的 Collision Monitor 默认开启描述以本文当前配置为准。

`collision_action_test.py` 已区分实车与仿真：默认 `--runtime sim` 使用 `/odometry`／Odometry，默认巡航触发、两程前进并持续运行；`--runtime real` 使用 `/motion_state`／MotionState，默认周期调度、去程前进返程倒车、四程。实车必须显式提供 `--a X Y --b X Y`，工具不替现场选择路线。`--feedback-topic` 可覆盖话题，反馈类型由 runtime 决定；`--odom-topic` 只供仿真兼容使用。

`--schedule periodic` 选择独立周期调度，发布调用目标频率为 50 Hz；五态保持时间默认仿真为 4／3／3／3／3 秒，实车为 12／4／4／3／3 秒，可用 `--state-durations` 覆盖。实际时序、各方向／状态响应和同一 Action 停后恢复分别记录，未覆盖的观察项标为 UNAVAILABLE。

截至本次文档核对，隔离真实节点配合合成反馈的测试、当前地图仿真测试、实车验证是不同证据层级。周期仿真仅取得部分减速／停止有效证据；独立状态源启停验收因 5 秒起步窗内一直转向而 UNAVAILABLE，断流停车和同任务恢复的完整两程验收仍未完成。历史原始报告由外层项目维护，本文不替代报告。

## 日志与排查

统一诊断默认文件快照自主模式 50 Hz、固定路径／remote 100 Hz，终端摘要 1 Hz，默认 `/tmp/nav2_logs`；可用 `log_dir` 指向数据盘。ROS Launch 日志由 ROS_LOG_DIR 管理。`enable_velocity_diagnostics` 默认 true，`enable_legacy_velocity_logs` 默认 false。详情见[速度日志](nav2_regulated_modules/doc/velocity_logging.md)。

`velocity … schema=2` 是各话题最新值的周期快照，同一话题／类型的多个角色会合并显示。`count` 重复表示没有新消息，`age` 是单调时钟计算的接收年龄；`status=valid` 只说明已收到、速度数值有限且未过期，不验证速度是否物理合理，也不证明停稳。各话题回调不是严格同步采样，不能把单行内的指令／反馈差值直接当作同一时刻的控制误差；`wz` 对应车体角速度，`w_shelf` 是另一物理量。

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /regulated_navigator
ros2 param get /regulated_navigator operation_mode
ros2 param get /controller_server odom_topic
ros2 param get /velocity_smoother odom_topic
ros2 param get /velocity_smoother collision_state_enabled
ros2 topic info /collision_state --verbose
```

UART 适配节点在有效输入到来后才发布，空闲时可能静默；零输出、真实反馈停稳、定位质量及任务状态需分别核查。配置改变不自动更新已启动进程。

## 全部包索引

以下按 package.xml 的包名列出，嵌套目录保持真实结构。

| 包 | 类型与职责 | 构建状态 |
|---|---|---|
| [aws_robomaker_small_warehouse_world](aws_robomaker_small_warehouse_world/README.md) | 资源包：提供 AWS 小型仓库世界、模型与地图资源，用于仿真场景搭建；不实现导航控制器。 | colcon 可发现 |
| [byd_custom_msgs](byd_custom_msgs/README.md) | 接口包：定义底盘命令、运动反馈、分段路径和固定路径 Action，是规控与底盘之间的消息契约。 | colcon 可发现 |
| [controlpub](controlpub/README.md) | 适配节点：将 Twist 速度输入转换为 ControlRes，供外部底盘通信链路使用；本包不实现串口驱动。 | colcon 可发现 |
| [costmap_queue](nav2_dwb_controller/costmap_queue/README.md) | 算法库：提供代价地图栅格按距离展开的队列工具，服务于 DWB 距离场计算。 | colcon 可发现 |
| [dwb_core](nav2_dwb_controller/dwb_core/README.md) | 控制插件与核心库：实现 DWB 局部控制框架，组合轨迹生成器与评分器选择速度指令。 | colcon 可发现 |
| [dwb_critics](nav2_dwb_controller/dwb_critics/README.md) | 轨迹评分插件：提供障碍、路径、目标、朝向及振荡等评分器，由 DWB Controller 装载。 | colcon 可发现 |
| [dwb_msgs](nav2_dwb_controller/dwb_msgs/README.md) | 接口包：定义 DWB 轨迹、评分与局部规划评估相关消息和服务。 | colcon 可发现 |
| [dwb_plugins](nav2_dwb_controller/dwb_plugins/README.md) | 轨迹生成插件：提供 DWB 标准及受加速度约束的轨迹生成器，供 dwb_core 采样使用。 | colcon 可发现 |
| [laserpub](laserpub/README.md) | 测试节点：发布可配置的合成二维 LaserScan，供隔离功能验证使用；不能替代真实激光观测。 | colcon 可发现 |
| [map2base_tf](map2base_tf/README.md) | 测试节点：周期发布固定位置的 map→base_link 动态 TF，供隔离接口验证；不实现定位算法。 | colcon 可发现 |
| [myagv_keyboard_control](myagv_keyboard_control/README.md) | 遥控节点：读取终端按键，按速度、加速度和 jerk 约束生成 ControlRes 底盘命令。 | colcon 可发现 |
| [mytb3_bringup](mytb3_bringup/README.md) | 仿真启动包：组织 TurtleBot3 的 Ignition Gazebo、机器人描述、传感器桥接、定位与导航场景。 | colcon 可发现 |
| [myworld_bringup](myworld_bringup/README.md) | 仿真启动包：组织仓库场景仿真与四模式装配；当前 entry 使用外部定位和系统时间适配。 | colcon 可发现 |
| [myworld_bringup_sim](myworld_bringup_sim/README.md) | 仿真启动包：保留另一套 myworld 场景启动、定位与导航配置；入口行为独立于 myworld_bringup。 | colcon 可发现 |
| [nav2_amcl](nav2_amcl/README.md) | 定位节点：使用粒子滤波、运动模型与激光地图匹配估计二维位姿，向导航提供定位变换。 | colcon 可发现 |
| [nav2_behavior_tree](nav2_behavior_tree/README.md) | 行为树插件库：提供规划、控制、恢复、条件与流程控制 BT 节点，供 BT Navigator 装载。 | colcon 可发现 |
| [nav2_behaviors](nav2_behaviors/README.md) | 行为服务器：通过行为插件提供旋转、后退、等待、按航向行驶及辅助遥控等动作。 | colcon 可发现 |
| [nav2_bringup](nav2_bringup/README.md) | 标准启动包：提供标准 Nav2 的导航、定位、组合节点和示例仿真启动资源。 | colcon 可发现 |
| [nav2_bt_navigator](nav2_bt_navigator/README.md) | 导航协调节点：执行行为树，将导航 Action 编排为规划、控制与恢复任务；与本项目无 BT 协调器是不同入口。 | colcon 可发现 |
| [nav2_collision_monitor](nav2_collision_monitor/README.md) | 速度过滤节点：依据 LaserScan、PointCloud2、Range 等传感器与配置区域，对输入速度实施停车、减速或接近保护。 | colcon 可发现 |
| [nav2_common](nav2_common/README.md) | 构建与启动工具库：提供 Nav2 通用 CMake 宏与 Python Launch 参数重写工具，包括 RewrittenYaml。 | colcon 可发现 |
| [nav2_constrained_smoother](nav2_constrained_smoother/README.md) | 平滑插件：通过 Ceres 优化路径的平滑性、曲率和障碍物约束，由 Smoother Server 加载。 | colcon 可发现 |
| [nav2_controller](nav2_controller/README.md) | 控制服务器：实现 FollowPath Action，加载控制器、进度检查器和目标检查器，计算并发布速度。 | colcon 可发现 |
| [nav2_core](nav2_core/README.md) | 插件接口库：定义 GlobalPlanner、Controller、Smoother、Behavior、GoalChecker 等 Nav2 插件基类。 | colcon 可发现 |
| [nav2_costmap_2d](nav2_costmap_2d/README.md) | 代价地图库与节点：组合静态、障碍、体素、膨胀等层及代价过滤器，为规划与控制提供环境代价。 | colcon 可发现 |
| [nav2_dwb_controller](nav2_dwb_controller/nav2_dwb_controller/README.md) | 元包：聚合 DWB 控制器相关包的依赖，不提供独立运行节点。 | colcon 可发现 |
| [nav2_graceful_controller](nav2_graceful_controller/README.md) | 控制插件：通过平滑控制律跟踪路径，由 Controller Server 加载并生成速度。 | colcon 可发现 |
| [nav2_lifecycle_manager](nav2_lifecycle_manager/README.md) | 生命周期管理节点：按配置顺序启动、激活、停用和监视生命周期节点，并通过 bond 检测连接状态。 | colcon 可发现 |
| [nav2_map_server](nav2_map_server/README.md) | 地图服务节点：提供地图加载、发布、保存及 Costmap Filter 信息服务。 | colcon 可发现 |
| [nav2_mppi_controller](nav2_mppi_controller/README.md) | 控制插件：通过模型预测路径积分采样控制序列，并按 critics 评分选择路径跟踪动作。 | colcon 可发现 |
| [nav2_msgs](nav2_msgs/README.md) | 接口包：定义标准 Nav2 导航、规划、路径跟踪、平滑、行为及生命周期相关消息、服务和 Action。 | colcon 可发现 |
| [nav2_navfn_planner](nav2_navfn_planner/README.md) | 规划插件：提供栅格 Dijkstra／A* 路径搜索，由 Planner Server 加载。 | colcon 可发现 |
| [nav2_planner](nav2_planner/README.md) | 规划服务器：实现 ComputePathToPose／ComputePathThroughPoses Action，并管理全局规划插件与全局代价地图。 | colcon 可发现 |
| [nav2_regulated_modules](nav2_regulated_modules/README.md) | 规控协调与插件包：提供 remote、autonomous、fixed_path、autonomous_fixed 四模式入口、无 BT 任务协调、固定路径插件和诊断工具。 | colcon 可发现 |
| [nav2_regulated_pure_pursuit_controller](nav2_regulated_pure_pursuit_controller/README.md) | 控制插件：提供受曲率、代价与目标接近约束的 Pure Pursuit 路径跟踪控制器。 | colcon 可发现 |
| [nav2_rotation_shim_controller](nav2_rotation_shim_controller/README.md) | 控制插件：在需要时先对齐路径朝向，再把控制交给配置的内部控制器。 | colcon 可发现 |
| [nav2_route](nav2_route/README.md) | 路网服务器与插件：从路网图搜索路线，并通过边评分和路线操作插件处理路线任务；不同于栅格 Planner Server。 | colcon 可发现 |
| [nav2_rviz_plugins](nav2_rviz_plugins/README.md) | RViz 插件：提供导航目标工具、Nav2 面板与粒子云显示，供 RViz 加载。 | colcon 可发现 |
| [nav2_simple_commander](nav2_simple_commander/README.md) | Python 客户端库：提供 BasicNavigator 及导航、路径跟踪、路点任务的 Python 示例与应用演示。 | colcon 可发现 |
| [nav2_smac_planner](nav2_smac_planner/README.md) | 规划插件：提供二维 A*、Hybrid-A* 和 State Lattice 搜索，支持不同车辆运动约束。 | colcon 可发现 |
| [nav2_smoother](nav2_smoother/README.md) | 平滑服务器与插件：实现 SmoothPath Action，加载简单平滑与 Savitzky–Golay 等路径平滑插件。 | colcon 可发现 |
| [nav2_system_tests](nav2_system_tests/README.md) | 系统测试资源：包含导航系统级测试脚本、场景和模型；当前 COLCON_IGNORE 使其不参加默认 colcon 构建。 | 默认忽略 |
| [nav2_theta_star_planner](nav2_theta_star_planner/README.md) | 规划插件：提供 Theta* 任意角路径搜索，通过栅格视线检查减少路径方向约束。 | colcon 可发现 |
| [nav2_util](nav2_util/README.md) | 公共运行库：提供生命周期节点、Action Server、TF、里程计、几何、速度及 collision 状态等公共工具。 | colcon 可发现 |
| [nav2_velocity_smoother](nav2_velocity_smoother/README.md) | 速度平滑节点：处理控制速度输入，执行开环／闭环平滑及本项目可选 jerk 限制与 collision 目标门控。 | colcon 可发现 |
| [nav2_voxel_grid](nav2_voxel_grid/README.md) | 体素算法库：提供按位压缩的三维体素标记、清除与射线追踪结构，服务于 VoxelLayer。 | colcon 可发现 |
| [nav2_waypoint_follower](nav2_waypoint_follower/README.md) | 路点服务器：把路点列表编排为导航任务，并在路点执行等待、拍照或输入触发等插件。 | colcon 可发现 |
| [nav_2d_msgs](nav2_dwb_controller/nav_2d_msgs/README.md) | 接口包：提供二维位姿、路径、速度等公共消息类型，供 DWB 系列使用。 | colcon 可发现 |
| [nav_2d_utils](nav2_dwb_controller/nav_2d_utils/README.md) | 工具库：提供二维导航消息转换、路径处理和 TF 辅助函数。 | colcon 可发现 |
| [navigation2](navigation2/README.md) | 元包：聚合 Nav2 栈依赖；源码仓库名称 navigation2 与本目录中的同名 ROS 元包应区分。 | colcon 可发现 |
| [spdlog_wrapper](spdlog_wrapper/README.md) | 日志库：封装 spdlog、YAML 配置与 ROS 日志转接，供规控与底盘适配等模块复用。 | colcon 可发现 |
| [teb_local_planner](teb_local_planner/README.md) | 控制插件：通过 Timed Elastic Band 优化轨迹时间、障碍距离与运动学约束，作为 Nav2 控制插件使用。 | colcon 可发现 |
| [teb_msgs](teb_msgs/README.md) | 接口包：定义 TEB 优化轨迹、反馈和障碍物消息，供 teb_local_planner 使用。 | colcon 可发现 |
| [turtlebot3](navigation2_ignition_gazebo_turtlebot3/turtlebot3/README.md) | 仿真资源包：安装 Ignition Gazebo TurtleBot3 的 Launch、地图、模型和参数；ROS 包名为 turtlebot3。 | colcon 可发现 |

## 外部贡献：Fork＋Pull Request

本仓库公开地址：

```text
https://github.com/zpy560/mynavigation2
```

普通外部贡献者不需要本仓库的写权限。推荐使用“Fork 到个人账号、在个人 Fork 开发、向本仓库
`main` 分支提交 Pull Request”的方式贡献代码。

### Fork仓库

贡献者登录 GitHub 后打开：

```text
https://github.com/zpy560/mynavigation2/fork
```

选择自己的个人账号并创建 Fork。完成后，贡献者会得到：

```text
https://github.com/CONTRIBUTOR_ACCOUNT/mynavigation2
```

其中 `CONTRIBUTOR_ACCOUNT` 需要替换为贡献者自己的 GitHub 用户名。

### 克隆个人Fork并添加上游仓库

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

### 创建开发分支

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

### 修改、验证并提交

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

### 推送个人分支

```bash
git push -u origin fix/map-server-lifecycle
```

贡献者只向自己的 `origin` 推送，不需要也不应直接向 `zpy560/mynavigation2` 的 `main` 分支推送。

### 创建Pull Request

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

### 根据审查意见更新PR

如果维护者提出修改意见，贡献者继续在同一个开发分支修改并推送即可：

```bash
git add PATH_TO_CHANGED_FILE
git commit -m "fix: 根据审查意见修正具体问题"
git push
```

新的提交会自动追加到现有 PR，不需要重新创建 PR。

### 同步上游main分支

当 PR 开发期间上游 `main` 有新提交时，可以把上游更新合并到当前开发分支：

```bash
git fetch upstream
git switch fix/map-server-lifecycle
git merge upstream/main
git push
```

如果出现冲突，应在本地解决冲突、重新验证后再推送，不要在不了解影响时覆盖上游文件。

### 权限边界

- 外部贡献者可以 Fork 公开仓库并提交 PR。
- 外部贡献者默认不能直接推送本仓库，也不能自行合并 PR。
- 仓库维护者负责审查、要求修改、批准、关闭或合并 PR。
- 长期可信任的协作者可以单独授予仓库写权限，但普通外部贡献优先使用 Fork＋PR。
- PR 被合并前，变更不会进入本仓库的 `main` 分支。
