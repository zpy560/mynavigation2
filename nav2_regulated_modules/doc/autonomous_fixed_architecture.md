# autonomous_fixed：一次规划与固定路径执行

`operation_mode:=autonomous_fixed` 是第四种启动模式。`remote`、`autonomous`、`fixed_path` 的入口与参数覆盖保持。新模式接收 `NavigateToPose` 和 `goal_pose`，拒绝多目标 Action、自定义行为树及固定路径 NavigationService。

## 执行约束

每个目标先等待新鲜定位、旧控制终态和连续停稳，再发送至多一次 `ComputePathToPose`。导航器和 Planner Server 都选择 `SmacHybrid`，运动模型为 `REEDS_SHEPP`。规划失败或超时直接失败；周期检查、定位恢复和控制恢复都不能重新调用规划器。

本模式只加载 `SmacHybrid`，距离启发式查找表窗口 `lookup_table_size=10.0` m，减少启动时的预计算开销。查找窗口不限制整条路径长度；地图分辨率、72 个方向和前进／倒车能力保持。配置仅在本模式的 Launch 参数生成阶段覆盖，共享 YAML 和其余模式保持原配置。启动提速与规划性能尚未运行验证。

本模式的全局／局部 Costmap 统一使用以 `base_link` 为中心、x 为车长方向的 0.8 m×0.6 m 矩形包络：`[[0.4,0.3],[0.4,-0.3],[-0.4,-0.3],[-0.4,0.3]]`。`footprint_padding=0.0`，保证发布和检查的矩形尺寸就是 0.8 m×0.6 m；生成参数移除圆形 `robot_radius`，由多边形足迹生效。Hybrid A* 使用全局包络，跟踪与制动预测使用局部包络，RViz 沿用两路 `published_footprint` 显示。

“距障碍增大一倍”在本次配置中落实为全局／局部 `inflation_layer.inflation_radius` 从 0.45 m 调至 0.90 m；`cost_scaling_factor` 从 5.0 调至 2.5，让代价随距离衰减更缓，增加规划远离障碍的倾向。膨胀半径是代价影响范围，不是矩形边界与障碍的硬性最小间距，参数倍增不能证明实际轨迹最小间距恰好倍增；实际效果需后续运行测量。新包络会改变狭窄通道可行性，无法通过时仍按原规则等待／失败，不缩小包络或清除真实障碍。

这些覆盖仅在 `NavigationRewrittenYaml.fixed_parameters()` 中应用，两个共享 YAML、场景入口、其余三模式和 Collision Monitor 开关保持；未调整车辆模型或定位。2026-10-30 本轮只修改与编译，不执行 Launch 解析测试、后台或运动验证。

Smac 内部平滑属于这一次请求；关闭额外 `SmoothPath`。输出通过坐标系、数值、姿态、连续性、方向及请求目标尾点验收后冻结。完整位置／姿态序列及 FNV-1a 诊断标识保存在任务中，完整路径发布到 `autonomous_fixed/plan`（Reliable、Transient Local）。路径标识不包含发布时间，不能作为安全认证。

### 原始目标的 10 mm 位置预算

仅本模式的 `SmacHybrid` ID 改用 `nav2_regulated_modules::AutonomousFixedHybridPlanner`，继承并调用原 `SmacPlannerHybrid::createPlan()` 一次。原规划器会把目标映射到栅格中心；之前长任务中，原始目标与冻结尾点相差约 10.67～16.41 mm，单独缩小控制验收容差不能消除这项偏差。

包装在搜索与内部平滑后、导航器冻结前，按“原始目标位置 − 原尾点位置”将所有路径点等量平移。曲率、弧长、方向和换向点相互关系保持，姿态序列不变，不追加尾段或连接线。偏移不得超过 3 cm，且不得超过半格对角线加本次搜索容差；超过即失败。使用当前全局 Costmap 的实际多边形足迹重新检查所有点，包含实际起点与移后首点间的检查区间；按平移和旋转引起的包络位移插值，采样间隔不超过 5 mm 或半个栅格。未知、致命代价、中心内切膨胀碰撞、越界或过期地图拒绝；搜索与检查共用 3.5 s 截止时间，不重试。原 Smac 调试 raw path 仍是校正前数据，规划服务器输出与冻结话题为校正后数据。

原始目标的最终 XY 容差默认并最多为 0.01 m；更严配置可保留。默认预算为规划尾点相对原始目标≤0.003 m、控制包装相对尾点≤0.007 m，协调器最终仍直接复核原始目标≤0.01 m，不能用路径尾点替换任务目标。刚性校正后的尾点通常与原始目标重合，但冻结入口仍保留 3 mm 预算检查。原规划只返回同栅格单点且需要平移时拒绝，避免凭空构造缺少切线的路径；短距离任务可行性尚未运行验证。

第四模式停车进入半径为 0.005 m，末端接近速度为 0.01 m/s；末端前视基础值 0.08 m、范围 0.05～0.12 m，参考速度 0.1 m/s、速度增益 0.2 s。常规速度、制动约束、原路径走廊、微调预算、定位和障碍门控保持。10 mm 是定位坐标系下的成功判据和调参目标，实际车辆精度仍取决于定位、执行和停车效果；本轮不运行测试或运动验证。

按前进／倒车换向和同位置旋转分段，当前段通过 `FollowPath` 交给 `AutonomousFixedPathController`。包装组合原 `FixedPathController`，启用 0.5 m 弧长范围的最近点匹配，避免自交路径跳段。原固定路径控制器新增的搜索范围默认值为 0，原模式仍搜索整个剩余路径，不改变其控制参数与默认行为。

前视点不跨换向段。每段先制动停稳；持续位置超差时进行有界末端微调，再校正该段末点车头姿态，连续复核通过后交接下一段。倒车微调仍输出负线速度，朝向按倒车运动学计算。最后一段还必须满足原始请求目标的位姿和停稳复核。

## 完成与暂停

- 终点锁停后先保持零速，实测线速度 ≤ 0.01 m/s、角速度 ≤ 0.05 rad/s 连续 0.2 s，才允许姿态对齐／验收；姿态对齐完成后也先确认停稳。随后位置、航向和速度连续合格 0.5 s，才报告完成。新包装不进入 Controller Server 的 2 秒宽松成功分支。
- 捕获范围内的短暂 XY 超差保持零速并重置连续验收计时。停稳后持续超差 0.2 s、且收到不同源时间戳的定位帧，非旋转段才可进入 TRIM_ALIGN／POSITION_TRIM；不能微调的情况仍受 2 s 位置窗口约束，超时不会当作成功。
- 微调目标始终是冻结原段尾点，只在原段末 0.20 m 定义的 0.02 m 走廊内收敛，不新增、改写或发布路径，不越过段尾平面。先对齐到段尾点的行驶朝向，朝向误差≤0.08 rad、实测角速度≤0.08 rad/s 后才允许平移；方向沿用原段。微调速度上限 0.02 m/s，名义低速下限 0.005 m/s，外部限速、加速和制动约束可使实际指令更低。
- 每次下发的子路径执行最多微调 2 次，单次实测平移≤0.02 m、累计≤0.03 m；从第一次微调开始使用同一个单调时钟，5 s 后仍需修正则失败。航向对齐后的再次微调共享原截止时间、次数和位移预算。预算含微调后的制动滑行，定位变化不能重置预算；反馈中断超过 0.2 s 后拒绝继续消费未知运动区间。
- 微调进入验收半径内部并预留速度相关停车余量后，重新 BRAKE→ALIGN_GOAL→BRAKE／SETTLE；仍需连续 0.5 s 满足原精度和速度条件。旋转后位置再超差只能在剩余预算内重试。
- 捕获范围为当前验收半径外再加 0.02 m：换向点合计 0.03 m、默认最终控制尾点合计 0.027 m。此范围不扩大成功容差；微调还需满足更窄的原路径走廊、原方向、段尾平面、实测预算与碰撞检查。超限直接失败，不生成连接路径。
- 最终位置相对原始请求目标≤0.01 m，控制包装对规划尾点≤0.007 m；请求航向容差约 9°，控制包装对规划尾点采用约 4.5°，协调器对原始目标再次检查。换向点保持 0.01 m／约 5°。
- 包装检查局部 Costmap 的前进、倒车、旋转及制动足迹。阻挡时输出零速度，连续可通行 0.5 s 后恢复；连续阻挡超过 60 s 失败。Collision Monitor 默认关闭，速度平滑器直接输出 `cmd_vel`；已有外部碰撞状态门控仍默认启用，由独立参数 `enable_collision_state` 控制。
- 行走期间由控制包装统一判断定位是否可用，导航器不再发布重复 TF 门控或三帧零速。首次取得可用定位、旋转段、终点制动锁存和距当前段终点 0.5 m 内，仍使用 `localization_timeout=0.3` s；沿途允许 TF 年龄至 `tracking_localization_timeout=0.6` s。终点严格区域还不小于任务速度上限乘沿途容忍时间再加 0.02 m 捕获余量。使用真实源时间戳和已有位姿，不重写 TF、不外推位置；单调计时异常、恢复时源时间戳倒退、超过 0.05 s 的未来 TF、查询不可用及速度反馈失效仍输出零速。
- 暂停期间保留同一个 FollowPath、原段游标和跟踪状态；至少两个不同源时间戳的可用定位帧且连续有效 0.15 s、反馈新鲜、位姿连续、距原段不超过 0.20 m，才允许继续。位姿连续性使用源时间间隔与速度上限构造运动上界，额外位置／航向余量为 0.05 m／0.1 rad；明显反向运动也转入完整恢复。重新输出前仍执行足迹与制动预测检查。
- `short_pause_timeout=0.5` s 只计算连续失效时间，收到可用数据即清除此计时，恢复稳定等待不消费该窗口。导航器以控制包装的暂停状态为准，不因自身 TF 刚过 0.3 s 或总暂停超过 0.5 s 取消 Action；一次保留 Action 的暂停仍受 10 s 恢复上限与 2 s 状态心跳上限约束。持续失效超过短时窗口或位姿不连续时，包装报告 WAIT_LOCALIZATION，才取消旧控制并进入完整恢复；该分支保留定位连续稳定 0.5 s、车辆停稳和旧执行结束要求，仅重新下发原冻结段，不重新规划。
- 短时恢复按实测速度初始化衔接，线速度幅值增加限制为 0.5 m/s²、角速度幅值增加限制为 0.8 rad/s²；降低限速、零速保护立即生效。暂停重置连续制动／到点验收计时，位置复核窗口、微调次数、位移和截止时间不重置。微调过程中若反馈缺失导致运动预算不可核算，仍按原规则失败。
- 恢复只截取原路径采样点，不加入当前位置或连接曲线，不跳过换向点。新目标抢占旧任务后，也要等待旧控制终态与停稳。
- 不可恢复控制错误、超出捕获范围、位置复核超时、状态反馈超时或无进展会失败，不清空真实障碍，也不生成新路径。

内部状态话题 `autonomous_fixed/execution_status` 使用 `std_msgs/String`，保留 `execution_id`、`point_index`、`phase`，新增 `reason`、`tf_stamp_ns`、`tf_age`、`odom_age`、`receive_age`、`pause_duration`。导航器同时接受原三字段格式和完整扩展格式。`execution_id` 对应当前下发子路径的唯一 Header 时间戳，`point_index` 为该子路径内的单调进度；协调器关联其分段表得到全局段号。阶段新增 PAUSED_LOCALIZATION，表示保留 Action 的零速短时暂停；WAIT_LOCALIZATION 表示必须取消后完整恢复。其余阶段保持。迟到执行编号的状态不能改变当前任务。

`autonomous_fixed/localization_gate` 的内部发布／订阅已移除。行走中的 TF 判断只由控制包装执行；导航器仍在启动、完整恢复和分段验收时独立检查新鲜定位。TF 在严格门限与沿途门限之间且允许继续行走时，状态为 TRACKING、原因为 TF_DELAY_TOLERATED；实际过期、查询不可用、未来时间戳、里程计源时间戳过期、接收超时及无效速度仍分别记录原因。缺少可用导航器位姿时暂缓其几何进度更新，控制包装与状态心跳、恢复上限仍有效。

轨迹拒绝日志最多每秒一条，区分 Costmap 未更新、当前／反应阶段／运动预测／制动预测足迹、微调走廊，并记录首个拒绝预测位姿、预测时长、足迹代价、中心栅格索引和代价、实测／目标速度。中心栅格不等于造成足迹边界碰撞的具体障碍点；仍需结合地图和雷达核对。检查判据与障碍等待上限保持。

新控制包装参数为 `brake_stable_duration=0.2`、`settle_duration=0.5`、`terminal_review_timeout=2.0`、`terminal_capture_margin=0.02`。这些配置仅属于第四模式；当前运行中的旧插件需要在用户重启导航后加载新实现。

连续性参数为导航器 `autonomous_fixed_short_pause_timeout=0.5`、`autonomous_fixed_resume_stable_duration=0.15`，Launch 同步至包装的 `short_pause_timeout`／`resume_stable_duration`，稳定时间必须小于连续失效窗口。新增 `autonomous_fixed_tracking_localization_timeout=0.6` s 与 `autonomous_fixed_terminal_localization_distance=0.5` m，分别同步至包装的 `tracking_localization_timeout`／`terminal_localization_distance`；沿途时间不得小于严格 `localization_timeout`。包装的严格 `localization_timeout`、`resume_distance` 同步导航器既有参数，`resume_linear_acceleration=0.5`。0.5 m/s 常规上限、制动参数、0.01 m/s 末端接近速度和原有到点容差保持。

2026-10-30 连续性与末端接近调整：按主人最新要求修改源码并完成 nav2_regulated_modules 定向 Release 编译；收尾增量构建 17.9 s 通过。已做差异空白及原模式保护文件哈希核对，未执行回归测试、Launch 解析测试或运行态验证，未操作后台。不能由编译通过推断到点时间、连续性或成功率改善。

微调参数为 `terminal_trim_max_speed=0.02`、`terminal_trim_max_step=0.02`、`terminal_trim_max_distance=0.03`、`terminal_trim_timeout=5.0`、`terminal_trim_max_attempts=2`、`terminal_trim_confirm_duration=0.2`。速度、位移、时间和次数参数只能在上述硬上限内收紧。定位／障碍暂停会清空连续验收，微调预算不随暂停清零。实际混合换向效果尚未验证，本轮按用户要求只修改与编译，不执行测试或后台验证。

## 启动与配置

实车规控入口和 `myworld_bringup` 仿真均使用 `/motion_state` 的 `byd_custom_msgs/msg/MotionState`；实车 YAML 显式设置导航器 `velocity_odom_topic`，使停稳判断与速度观测同样使用 `v_car/w_car`。Gazebo 原始 `/odometry` 保留为转换输入，反馈源 Header 保持。控制包装及导航器按话题末段 `motion_state` 选择消息类型，其余反馈话题保留 Odometry 兼容；自定义参数文件省略该字段时仍沿用 C++ 默认 `/odometry`。消息只转换为内部速度快照，不提供定位位姿；速度反馈 0.2 s 时效与停稳阈值保持。

运行验证必须先在独立终端启动主人指定的定位模块，并保持运行：

```bash
cd /home/forlinx/lidar_slam/colcon_ws
ros2 launch cartographer_ros cartographer.launch.py
```

确认定位输入、TF 及时间源一致，再启动仿真：

```bash
cd /mnt/nvme/codex/nav2_demo/nav2_ws
source install/setup.bash
ros2 launch myworld_bringup entry.launch.py operation_mode:=autonomous_fixed
```

该场景入口默认启动 Gazebo；使用已有外部传感器与底盘输入时显式指定 `use_gazebo:=false`。不要将实车输入和仿真输入混合。图形显示沿用 `use_rviz`／`headless` 实参，新 RViz 配置显示冻结全局路径。

新模式专用导航器参数：`autonomous_fixed_max_speed=0.5`、`autonomous_fixed_xy_tolerance=0.01`、`autonomous_fixed_yaw_tolerance=0.15726646259971647`、`autonomous_fixed_obstacle_timeout=60.0`、`autonomous_fixed_resume_distance=0.20`。均要求有限正数，XY 容差不得大于 0.01 m；Launch 会将旧配置中更宽的 XY 值收紧到 0.01 m，保留更严值。参数放在所选 YAML 的 `regulated_navigator.ros__parameters` 下；新模式 Launch 独立生成运行配置，不修改共享 YAML 文件。

控制与速度平滑均为 100 Hz；最大角速度 0.4 rad/s；Hybrid 最小转弯半径 0.40 m，默认搜索近似容差 0.003 m，规划与校正合计上限 3.5 s，请求超时 5 s。实际速度继续受外部限速、曲率和制动限制。

## 验证边界

定向构建在 `nav2_ws` 中进行，不在源码仓库运行 colcon。CTest 覆盖分段、方向、进度、路径标识、非法输入、参数隔离及既有规控策略。`test/autonomous_fixed_runtime_probe.py` 不由 CTest 自动运行；只有显式传入 `--run-simulation-goals` 且检测到仿真桥接后才发送最多两个前进／倒车目标。

构建、单元测试、协议验证、Gazebo 和实车证据分别记录。实际换向、动态障碍、定位恢复、控制频率和目标成功率必须以运行记录为准，不能由静态检查推断。
