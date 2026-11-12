# nav2_regulated_modules

## 职责与结构

推荐的规控统一入口，提供无 BT 任务协调、规划／平滑／控制客户端、专用固定路径控制器、目标检查器、速度诊断与碰撞区域可视化。仿真世界和传感器桥接由场景包负责，外部定位、激光、轮速和底盘通信需另行提供。

| 目录／声明 | 内容 |
|---|---|
| [launch/](launch/) | `regulated_modules.launch.py` 与 RViz 入口 |
| [params/regulated_modules.yaml](params/regulated_modules.yaml) | 节点参数、规划／控制插件和反馈话题 |
| [src/](src/)／[include/](include/) | 协调器、规划与控制模块、固定路径跟踪与停止逻辑 |
| [fixed_path_controller_plugin.xml](fixed_path_controller_plugin.xml) | `nav2_regulated_modules::FixedPathController` 与 `FixedPathGoalChecker` |
| [autonomous_controller_plugin.xml](autonomous_controller_plugin.xml) | 仅自主配置选择的 MPPI 转向／接近包装和连续停稳 GoalChecker |
| [tools/README.md](tools/README.md)／[test/](test/) | 现有环境观察、隔离实验及开发验证 |
| [doc/](doc/) | 架构、日志与分开验收方法 |

依赖标准 Nav2 服务器、插件、`byd_custom_msgs`、`controlpub`、`myagv_keyboard_control` 及 ROS 2 通信库，完整依赖见 [package.xml](package.xml)，安装可执行目标见 [CMakeLists.txt](CMakeLists.txt)。

## 构建与启动

从项目根目录执行；两个工作空间不要叠加加载：

```bash
cd nav2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to nav2_regulated_modules \
  --cmake-args -DBUILD_TESTING=OFF
source install/local_setup.bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=autonomous
```

`BUILD_TESTING=OFF` 适用于运行构建，不执行测试。模式启动后不能热切换；停止旧入口后再切换。

| 模式 | 接受的任务输入 | 行为 |
|---|---|---|
| `remote` | 键盘终端 | 使用键盘包直接生成底盘 ControlRes；线／角速度默认限幅 0.3 m/s、0.3 rad/s |
| `autonomous`（默认） | `/navigate_to_pose`、`/navigate_through_poses`、`/goal_pose` | regulated_navigator 调用 Planner／可选 Smoother／Controller，无行为树 |
| `fixed_path` | `/navigation_service` | 由 NaviSegment[] 生成路径，使用专用控制器与目标检查器跟踪 |
| `autonomous_fixed` | `/navigate_to_pose`、`/goal_pose`；拒绝多点 Action | Hybrid A*／REEDS_SHEPP 一次规划，冻结路径后分段跟踪，暂停恢复不重规划 |

直接入口不启动 Gazebo、AMCL、合成 TF 或合成雷达。默认地图为 `/userdata/map/pbstream/out.yaml`；用 `map:=...` 覆盖。可用 `ros2 launch nav2_regulated_modules regulated_modules.launch.py --show-args` 查看安装版本支持的参数，实际值受 YAML、Launch 重写及显式实参共同影响。

默认 `fixed_path` 入口采用满载纵向运动限值：最终速度平滑输出的正常加速度／减速度幅值均不超过 0.3 m/s²，jerk 不超过 1.0 m/s³，覆盖前进／后退起步、途中变速、减速和换向；停车规划减速度仍为 YAML 的 0.2 m/s²。限制作用于指令，底盘实测加速度与上装振动仍需实车验收。当前不识别负重，空载固定路径同样使用此档。仅重写平滑器四项限值的 x 分量，保留角向、共享 YAML、投影计距及终点判据；autonomous／autonomous_fixed／remote 和显式 `adaptive_goal_braking_enabled:=true` 分支保持原配置。0→0.3 m/s 的理想斜坡约 1.3 s，高速加减速所需时间／距离也随之增加。

自定义参数文件中更严格的纵向限值继续沿用。当前 S 曲线以加／减速度幅值的较小值约束两向运动；若自定义停车规划减速度高于该有效限值，Launch 会明确报错，避免按无法达到的减速度估计停车距离。

场景入口的 `myworld_bringup/params/myworld2.yaml` 将导航器 `planner_id`、规划服务器 `selected_planner` 配置为 `SmacHybrid`，并设置 `SmacHybrid.motion_model_for_search=REEDS_SHEPP`（RS，允许前进／倒车搜索）。场景 Launch 显式传入空的 `autonomous_planner_id`，在 autonomous 中让这三个值以参数文件为准。直接启动本规控 Launch 时，`autonomous_planner_id` 默认 `SmacHybrid`，仅 autonomous 同步覆盖导航器 `planner_id` 和服务器 `selected_planner`；显式传空可沿用 YAML，fixed_path／remote 忽略该实参。autonomous 的运动模型读取所选 YAML，直接入口共享文件当前为 `DUBIN`；autonomous_fixed 则独立覆盖为 `REEDS_SHEPP` 和项目包装插件。已运行的导航栈需重新启动才加载文件改动。

自主模式专用修复包括 TF 源时间戳校验、有效旋转／末端误差进度、真正恢复后重建计时基准；fixed_path／remote 不进入这些新分支。Controller 的 PoseProgressChecker 和较小位移阈值也只在 autonomous 的参数重写中启用。场景入口额外传 `autonomous_mppi_profile=true`，在 `MPPI` ID 下选择 `AutonomousMPPIController`（内部为 MPPI），并选择 `AutonomousStoppedGoalChecker`；直接入口默认该开关 false，保留原 RPP 选择。共享 YAML、FixedPathController 和固定路径 GoalChecker 未改。

自主 MPPI 包装负责起步对齐、末端专用位置收敛、制动停稳和终点朝向对齐；目标检查器保留位置／航向／速度阈值并要求连续达标 0.5 s。自主模式的控制频率为 50 Hz，MPPI 模型步长 0.02 s、75 步预测 1.5 s，场景自主包装的候选数为 300，MPPI 前进速度上限为 0.5 m/s；关闭包装仍保留自主模式的 50 Hz／MPPI 时间网格。自主速度平滑器及默认诊断快照同为 50 Hz，固定路径模式保持原有 100 Hz。不据此声称实际达到 50 Hz 或已通过导航任务验收。实现见 [autonomous_mppi_controller.cpp](src/autonomous_mppi_controller.cpp) 。

## 实车四种模式的启动

每个终端先从 `nav2_ws` 加载环境，保持相同的 `ROS_DOMAIN_ID` 和 DDS 设置。实车使用系统时间；导航三模式要求外部激光、`/motion_state`、底盘通信和新鲜 `map→base_link` 已就绪，地图与定位坐标系一致。本入口不启动这些驱动，也不启动 Cartographer 或 AMCL。外部 Cartographer 在其实际工作空间环境中沿用 `ros2 launch cartographer_ros cartographer.launch.py`，不以本包代替定位。

```bash
cd /mnt/nvme/codex/nav2_demo/nav2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

以下四个入口互斥，按模式只运行一个；导航命令使用 `MAP_FILE`，在同一终端先设置实际可读地图路径：

```bash
MAP_FILE='/userdata/map/pbstream/out.yaml'
test -r "$MAP_FILE"
```

### remote：键盘直控

无需导航 Action 或地图规划，在可交互终端运行。按键、终端设备和停车行为见[键盘包 README](../myagv_keyboard_control/README.md#三运行方法)。

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=remote use_rviz:=false use_sim_time:=false
```

### autonomous：可重新规划的自主导航

直接入口默认 `SmacHybrid`＋`RPP`，不启用场景专用 MPPI 包装。单点、多点 Action 或 RViz 的单点目标均可使用。

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=autonomous use_rviz:=false use_sim_time:=false \
  map:="${MAP_FILE:?先设置实际地图路径}"
```

### fixed_path：业务分段固定路径

只接收 NavigationService；不通过 RViz 导航目标自动生成业务路径。

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=fixed_path use_rviz:=false use_sim_time:=false \
  map:="${MAP_FILE:?先设置实际地图路径}"
```

### autonomous_fixed：单点规划后冻结执行

默认使用项目 Hybrid 包装、REEDS_SHEPP、10 m 距离查表和专用固定路径执行链。接收单点 Action 或 RViz 单点目标，拒绝 NavigateThroughPoses、NavigationService 和非空 `behavior_tree`。

沿途由控制包装统一处理 TF 延迟，默认可用年龄上限 0.6 s；首次起步、旋转和终点附近仍按 0.3 s 严格检查。短暂停保留同一 FollowPath，连续失效计时与恢复稳定计时分开，导航器不重复发 TF 停车请求。参数及失效边界见 [暂停恢复说明](doc/autonomous_fixed_architecture.md#完成与暂停)；这不代表可脱离定位运行，实车连续性与终点精度仍需新日志验收。

```bash
ros2 launch nav2_regulated_modules regulated_modules.launch.py \
  operation_mode:=autonomous_fixed use_rviz:=false use_sim_time:=false \
  map:="${MAP_FILE:?先设置实际地图路径}"
```

## 仿真入口与实车入口的区别

完整的四模式仿真命令见 [myworld_bringup README](../myworld_bringup/README.md#四种模式的仿真启动)。场景入口负责 Gazebo、传感器桥接、速度反馈转换与机器人内部 TF，外部定位仍需另行启动；默认 `use_sim_time=false`、`headless=true`。不要同时再启动一份本包规控入口。

已有独立仿真及外部定位时，也可使用上面的直接规控命令，但须自行提供匹配的传感器、TF、反馈和参数文件，不能默认其配置与场景入口相同。场景 autonomous 使用 SmacHybrid／REEDS_SHEPP＋专用 MPPI，直接 autonomous 默认 SmacHybrid／DUBIN＋RPP；fixed_path 和 autonomous_fixed 也有模式专属的参数重写。场景入口的 `use_gazebo:=false` 仅关闭仿真及桥接，仍保留场景模型与参数，不自动切换为实车共享 YAML。

## 独立打开 RViz2

三个导航模式直接启动时默认 `use_rviz=false`。场景入口的导航模式默认 `use_rviz=true`；本文命令显式传 false，以便另一终端独立打开。remote 在两个入口都不进入导航 RViz 启动分支，不能仅看 `use_rviz` 的声明默认值判断是否会打开窗口。

另开终端，重复加载 ROS 2 和同一个 `nav2_ws` 环境，再按当前模式选择一个命令。以下均使用系统时间，匹配当前仿真与实车的默认设置：

```bash
rviz2 -d "$(ros2 pkg prefix --share nav2_regulated_modules)/rviz/autonomous.rviz" \
  --ros-args -p use_sim_time:=false
```

```bash
rviz2 -d "$(ros2 pkg prefix --share nav2_regulated_modules)/rviz/fixed_path.rviz" \
  --ros-args -p use_sim_time:=false
```

```bash
rviz2 -d "$(ros2 pkg prefix --share nav2_regulated_modules)/rviz/autonomous_fixed.rviz" \
  --ros-args -p use_sim_time:=false
```

remote 若需观察已有地图和传感器，可独立打开通用配置；缺少地图、定位或导航服务器时，相应显示／面板不可用，不影响键盘入口本身：

```bash
rviz2 -d "$(ros2 pkg prefix --share nav2_regulated_modules)/rviz/nav2_default_view.rviz" \
  --ros-args -p use_sim_time:=false
```

| 模式 | 配置文件 | 默认启用的模式路径显示 |
|---|---|---|
| `autonomous_fixed` | [autonomous_fixed.rviz](rviz/autonomous_fixed.rviz) | `/autonomous_fixed/plan` 冻结路径 |
| `autonomous` | [autonomous.rviz](rviz/autonomous.rviz) | `/plan`、`/local_plan` |
| `fixed_path` | [fixed_path.rviz](rviz/fixed_path.rviz) | `/fixed_path_plan`、`/fixed_path_boundaries` |

三份文件沿用原直接入口配置的视角、地图、TF、雷达、代价地图和工具面板，关闭其他模式专用路径的默认显示。固定路径和冻结路径的订阅使用 `Reliable`、`Transient Local`，发布端仍保留数据时，后打开 RViz 也可接收路径。原 `nav2_default_view.rviz` 保留。

配置按默认无命名空间使用。只有导航、定位、桥接等全部切换为仿真时间，并有 `/clock` 时，才把 RViz 的 `use_sim_time` 同步改为 true；不能仅因使用 Gazebo 就改为 true。此参数不随 `.rviz` 文件保存。使用命名空间时需按实际话题前缀调整配置、节点与 Action 名称。`.rviz` 只保存显示配置，不保存地图数据或实时消息。

在 `autonomous`／`autonomous_fixed` 中，将 Fixed Frame 设为 `map`，确认定位与生命周期就绪，再用工具栏 `Nav2 Goal` 点击目标并拖动终点方向。当前工具经 Nav2 面板发送 NavigateToPose；autonomous_fixed 不使用多点／waypoint 工作流。fixed_path 的 RViz 用于看业务路径，不把 Nav2 Goal 当作 NavigationService；remote 不下发导航目标。`2D Pose Estimate` 是否被采用取决于外部定位实现，不替代启动定位模块。

如需随导航打开 RViz，改用 `use_rviz:=true`，直接入口默认按模式选取本包配置，自定义文件使用 `rviz_config_file:=/absolute/path/view.rviz`；该 Launch 的 RViz 子入口没有传递 `use_sim_time`，因此仿真时钟场景优先使用上面的独立命令显式设置时间。场景入口自带 RViz 会传入所选时间参数，场景模式配置见[场景 README](../myworld_bringup/README.md#仿真-rviz2-与时间设置)。独立进程退出不触发导航 Launch 的 RViz 退出事件；直接规控内置 RViz 的现有退出联动保持。

## Action 下发与结果检查

### 下发前的只读检查

在已加载同一工作空间的终端确认模式、生命周期、反馈和 TF。remote 不启动导航器，以下导航节点检查仅适用于三个导航模式。Action 能被发现不代表当前模式会接受其 Goal。

```bash
ros2 param get /regulated_navigator operation_mode
ros2 lifecycle get /regulated_navigator
ros2 lifecycle get /controller_server
ros2 action list -t
ros2 topic info /motion_state --verbose
ros2 run tf2_ros tf2_echo map base_link
```

`tf2_echo` 持续输出，检查完成后结束该只读命令。传感器、TF 时间源与实际模式正确、节点 active 后再发送任务。同一时刻只运行一个任务示例，新目标可能抢占旧任务；不要把示例自动作为现场测试路线。

### 单点 Action：autonomous 与 autonomous_fixed

在同一终端先设置 `GOAL_X`、`GOAL_Y` 为已核对可通行的地图坐标，单位米。以下采用终点 yaw=0 的单位四元数；其他方向需填写合法的 `orientation.z/w`，不能直接填角度。两个模式使用同一消息类型，`behavior_tree` 均必须留空。

```bash
: "${GOAL_X:?先设置地图目标X}" "${GOAL_Y:?先设置地图目标Y}"
ros2 action send_goal --feedback /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, behavior_tree: ''}"
```

### 多点 Action：仅 autonomous

先设置中间点 `VIA_X`、`VIA_Y` 与终点 `GOAL_X`、`GOAL_Y`，下面两点均使用 yaw=0。NavigateThroughPoses 是有序目标序列，不等于要求每个中间点停站。autonomous_fixed 会拒绝此请求，即使 Action 出现在发现列表中。

```bash
: "${VIA_X:?先设置中间点X}" "${VIA_Y:?先设置中间点Y}"
: "${GOAL_X:?先设置地图目标X}" "${GOAL_Y:?先设置地图目标Y}"
ros2 action send_goal --feedback /navigate_through_poses \
  nav2_msgs/action/NavigateThroughPoses \
  "{poses: [{header: {frame_id: map}, pose: {position: {x: $VIA_X, y: $VIA_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}], behavior_tree: ''}"
```

### 固定路径 Action：仅 fixed_path

先设置已核对路线的 `START_X`、`START_Y`、`END_X`、`END_Y`，起点应与实际位姿相匹配。下面是一条前进直线，任务限速 0.20 m/s；实际速度还受起步、曲率、制动、collision 门控与平滑器限制。倒车将 `motion_direction` 改为 2，`max_speed` 仍为正数；同一任务各段须同向。

```bash
: "${START_X:?先设置路径起点X}" "${START_Y:?先设置路径起点Y}"
: "${END_X:?先设置路径终点X}" "${END_Y:?先设置路径终点Y}"
ros2 action send_goal --feedback /navigation_service \
  byd_custom_msgs/action/NavigationService \
  "{task_id: 'readme_line_001', navi_segment: [{segment_type: 1, segment_name: 'line', segment_id: 'segment_001', node1: {x: $START_X, y: $START_Y, z: 0.0}, node2: {x: $END_X, y: $END_Y, z: 0.0}, max_speed: 0.20, motion_direction: 1}]}"
```

类型与字段以 [NavigationService.action](../byd_custom_msgs/action/NavigationService.action)、[NaviSegment.msg](../byd_custom_msgs/msg/NaviSegment.msg)、[NavigateToPose.action](../nav2_msgs/action/NavigateToPose.action)、[NavigateThroughPoses.action](../nav2_msgs/action/NavigateThroughPoses.action) 为准。三次贝塞尔还需控制点，历史几何例子保留在[场景固定路径任务](../myworld_bringup/README.md#固定路径任务)，不作为当前地图的默认路线。

### 反馈、取消与停稳

`Goal accepted` 只表示接收，需继续等 `SUCCEEDED`、`ABORTED` 或 `CANCELED`。标准导航反馈有当前位置、剩余距离、耗时与预计剩余时间；多点另有剩余目标数。NavigationService 反馈为 `cur_task_id`、`cur_seg_id`、`progress`，成功结果还应核对 `finish`，不能混用两种协议。

本机 Humble 的 `ros2 action send_goal` 在 Goal 被接受后注册 SIGINT 处理器，原终端 `Ctrl+C` 会请求取消该 Goal。等待取消结果并确认 `/motion_state` 的实际速度停稳；关闭终端、断开客户端或取消请求被接受本身都不证明已经停稳。跨终端按 UUID 取消的方法见[仓库 README](../README.md#结果判断与取消任务)，NavigationService 使用自己的 UUID 和 `/navigation_service/_action/cancel_goal`，不使用全零 UUID 取消全部任务。RViz 面板的取消按钮针对其自己持有的任务句柄，不视为任意外部 Action 的全局停止按钮。

## 数据流与反馈

自主包装的起步对齐按 Controller Server 提供的同一帧机器人位姿计算路径朝向；不再调用原 RotationShim 为路径采样点写 `now()` 的变换链。同坐标系不查询 TF，跨坐标系按该位姿时间戳零等待查询，缺失时报告失败并停车；保留起步旋转足迹检查和原定位时效门限。此改动只作用于自主插件，固定路径、共享 `nav2_util` 和 RotationShim 实现保持。未来外推导致的默认 100 ms 等待被去除，实际控制周期仍需运行测量。

```text
autonomous: 导航目标 → regulated_navigator → Planner → 可选 Smoother → FollowPath
fixed_path: NavigationService → 方向化 Path → FollowPath
autonomous_fixed: 单点目标 → 一次 Hybrid 规划 → 冻结／分段 → FollowPath
FollowPath → Controller Server → /cmd_vel_nav
  → Velocity Smoother → /cmd_vel → controlpub → /control_to_uart
启用 Collision Monitor 时：平滑器 → /cmd_vel_collision_in → Collision Monitor → /cmd_vel
```

实车 YAML 和 `myworld_bringup` 仿真均将控制器、导航器速度观测及 `autonomous_fixed` 停稳判断、平滑器统一到 `/motion_state` 的 MotionState；导航器通过 `velocity_odom_topic` 显式配置。控制包装及导航器按反馈话题末段 `motion_state` 选择 MotionState，其余话题保留 Odometry 兼容；自定义参数文件若省略该字段，导航器仍沿用 C++ 默认 `/odometry`。`v_car/w_car` 映射到线／角速度并保留源 Header；反馈过期、定位缺失、取消和正常到点分别处理。TF 须提供新鲜 `map→base_link`，或由明确的 TF 链组合得到；不把原始里程计位置当作地图定位。

`regulated_navigator` 另监听 `/downstream/chassis_control`，收到该支路有效命令才处理底盘请求。部署时须核对最终控制源，避免键盘直控与导航转发同时竞争 `/control_to_uart`。

自主反馈以请求目标的直线距离作为 `distance_remaining` 下界，并估计平移、终点航向及停稳时间；ETA 不直接当作成功计时。定位恢复与控制恢复分别计数，标准反馈报告两者总数，原控制恢复次数上限不变。自主 GoalChecker 每秒提供终点等待原因与误差／阈值／稳定时长，成功仍只由 FollowPath 的成功结果向上传递。标准 Nav2 Action 消息定义和固定路径反馈均保持。

## 固定路径与终点行为

- [NavigationService](../byd_custom_msgs/action/NavigationService.action) 接收 `task_id` 和 [NaviSegment[]](../byd_custom_msgs/msg/NaviSegment.msg)。支持直线与三次贝塞尔；相邻段连续，同一 Action 所有段同方向。
- `max_speed` 为有限正数，所有段取最小值，再受 `fixed_path_max_speed=1.5` 限制。方向 1／2 分别生成前进／倒车车体姿态和正／负速度，不修改初始定位或 Gazebo 位姿。
- 起点总距离上限 1.2 m；首次横向误差决定直接跟踪或严格对齐策略。横向误差较小时仍需满足航向条件才进入直线跟踪，大航向误差可能先原地转向；收到 Action 不代表立即达速。
- 每条路径起步保护距离默认 0.5 m，速度上限 0.3 m/s；横向和航向误差满足条件并稳定后解除，当前 YAML 的释放航向门限为 20°、稳定周期 10。
- 当前项目 YAML 的停车入口 0.008 m 与目标检查器位置容差 0.01 m 分开；位置／终点平面及停稳参与终点判断，终点航向作诊断。内部成功不等于独立地图精度或实车精度通过。

启动与 Action 几何例子见[仿真入口说明](../myworld_bringup/README.md)；控制实现见 [fixed_path_controller.cpp](src/fixed_path_controller.cpp) 与 [fixed_path_goal_checker.cpp](src/fixed_path_goal_checker.cpp)。

## collision 状态与 Collision Monitor

两项功能独立：外部 `/collision_state` 门控接收 `std_msgs/msg/UInt8`；Collision Monitor 依据传感器区域过滤速度，本包不从激光自动生成外部 collision 状态。

| 项目 | 直接规控入口 | `myworld_bringup/entry.launch.py` |
|---|---|---|
| `enable_collision_state` | 默认 true，仅 fixed_path 生效 | 默认 true，仅 fixed_path 生效 |
| `use_collision_monitor` | 默认 false | 默认 false |
| `use_collision_visualization` | 默认 true，导航模式 | 仅 fixed_path 默认 true，其他模式 false |

`enable_collision_state` 由 Launch 转成三节点的 `collision_state_enabled`。项目 YAML 为 true；通用 C++ 节点独立运行的内置回退仍为 false，不能混写这两层默认值。自主和遥控模式不启用外部状态门控。

| 有效状态 | 含义 | 相对任务速度上限 |
|---|---|---:|
| 4 | 正常 | 100% |
| 3 | 一级减速 | 65% |
| 2 | 二级减速 | 35% |
| 1、0 | 停止 | 0 |

启动、configure 或 reset 后从未收到消息时为 4，保留 `received=false`、`age=-1`、`reason=initial`，不因没有首条消息而超时。首次收到后按实际值处理，非法值或接收间隔达到 0.5 s 按 0 平滑停车；超时使用单调时钟。订阅为 BEST_EFFORT／VOLATILE／KeepLast(1)。

状态 4 只解除限制，不创建任务或速度。停止期间暂停对应无进展计时；恢复时仅在原任务及速度输入仍有效时继续，不重建路径或恢复已取消任务。门控在 S 曲线目标生成前，要求 `jerk_limited_smoothing=true`；参数修改需要重启或生命周期重新配置。

已有有效任务时，可在同环境终端持续发布状态；下面命令可能使车辆按原任务运动：

```bash
ros2 topic pub --rate 50 /collision_state std_msgs/msg/UInt8 '{data: 4}'
```

周期例子见 [COLLISION_TEST.md](tools/COLLISION_TEST.md)：50 Hz 发布 `4→3→2→1→0`，默认保持 4／3／3／3／3 秒；请求 1.5 m/s，支持去程前进、返程倒车。有限运行示例（先核对地图端点与起步位姿）：

```bash
ros2 run nav2_regulated_modules collision_action_test.py \
  --schedule periodic --return-direction backward --max-legs 4 \
  --output-dir "$PWD/collision-runs/$(date +%Y%m%d-%H%M%S)"
```

该周期工具退出时会发布停止并取消自身任务，不能用它的 Ctrl+C 代替“独立状态源断流、原 Action 保持”的验收。配置 50 Hz 与实测发布调用频率、独立订阅接收频率应分别报告。

## 日志、工具与证据

统一速度诊断默认开启：自主模式文件快照 50 Hz、固定路径／remote 100 Hz，终端摘要 1 Hz；默认日志目录 `/tmp/nav2_logs`，可通过 `log_dir` 改到数据盘。`ROS_LOG_DIR` 管理 ROS Launch 日志，两者独立。`enable_legacy_velocity_logs=false` 默认抑制重复摘要，关闭统一诊断后回退旧记录。

- [速度日志](doc/velocity_logging.md)：字段、异步滚动、S 曲线及缺失／过期状态。
- [固定路径 TF 诊断](doc/tf_diagnostics.md)：map→base_link 默认 100 Hz 采样打印、直接边逐条接收打印、控制器消费记录及逐边／时钟摘要。
- [固定路径架构](doc/fixed_path_architecture.md)：既有模块与控制流程分析，其中旧的 Collision Monitor 默认开启描述以本 README 的当前默认表为准。
- [分开验收](doc/action_only_acceptance.md)：速度停稳与定位采样质量分开评价。
- [测试工具](tools/README.md)：工具运行前提及会修改环境的实验边界。

本文为当前源码静态说明。本轮文档更新不执行构建或运动。已有隔离真实节点＋合成反馈回归不等同于实车验收；现有短路线周期测试只取得部分减速／停止证据，独立状态源启停验收因起步转向占满 5 秒基准窗而 UNAVAILABLE，不能声称两程全部通过。动态验证记录由外层项目 ROADMAP 与报告维护。

## autonomous_fixed：第四种运行模式

新增 Hybrid A*／REEDS_SHEPP 一次规划、路径冻结和固定路径分段跟踪。先启动指定 Cartographer 定位，再启动 `operation_mode:=autonomous_fixed`。支持单点目标、倒车和停稳换向，暂停后沿原路径恢复；不执行重规划。接口、参数及验证边界见 [自主固定路径架构](doc/autonomous_fixed_architecture.md) 。

第四模式保留 0.5 s 短时定位暂停窗口，满足不同源时间戳及连续有效 0.15 s 等条件后沿同一个 FollowPath 恢复；持续失效仍取消并完整恢复。位置验收相对原始请求目标收紧到 10 mm，内部控制容差 7 mm、停车进入半径 5 mm，末端接近速度 0.01 m/s；停稳要求保持，碰撞监控默认关闭。仅代码与编译交付，未运行测试或后台验证，实际精度未验证。

仅第四模式的 `SmacHybrid` ID 装载 `AutonomousFixedHybridPlanner`：复用一次 Hybrid A*，在冻结前将整条路径做有界等量平移以消除终点格心偏差，保持曲率、方向和换向点关系，并重新检查全路径矩形足迹及扫掠采样。超出 3 cm 偏移上限、碰撞、超时或同栅格单点无法表示平移时直接失败，不追加路径或重规划。其它模式仍使用原规划器。

仅第四模式的全局／局部 Costmap 使用长 0.8 m、宽 0.6 m 的矩形足迹（无额外 padding），膨胀半径 0.45→0.90 m、代价衰减系数 5.0→2.5。配置目标是增加路径与障碍间距；实际轨迹间距是否达到两倍需后续测量。共享 YAML 与其余模式保持原配置，本轮不运行测试。
