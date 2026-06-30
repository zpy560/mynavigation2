# Constrained Smoother

A smoother plugin for `nav2_smoother` based on the original deprecated smoother in `nav2_smac_planner` by [Steve Macenski](https://www.linkedin.com/in/steve-macenski-41a985101/) and put into operational state by [**RoboTech Vision**](https://robotechvision.com/). Suitable for applications which need planned global path to be pushed away from obstacles and/or for Reeds-Shepp motion models.

See documentation on navigation.ros.org: https://navigation.ros.org/configuration/packages/configuring-constrained-smoother.html


Example of configuration (see indoor_navigation package of this repo for a full launch configuration):

```
smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["SmoothPath"]

    SmoothPath:
      plugin: "nav2_constrained_smoother/ConstrainedSmoother"
      reversing_enabled: true       # whether to detect forward/reverse direction and cusps. Should be set to false for paths without orientations assigned
      path_downsampling_factor: 3   # every n-th node of the path is taken. Useful for speed-up
      path_upsampling_factor: 1     # 0 - path remains downsampled, 1 - path is upsampled back to original granularity using cubic bezier, 2... - more upsampling
      keep_start_orientation: true  # whether to prevent the start orientation from being smoothed
      keep_goal_orientation: true   # whether to prevent the gpal orientation from being smoothed
      minimum_turning_radius: 0.40  # minimum turning radius the robot can perform. Can be set to 0.0 (or w_curve can be set to 0.0 with the same effect) for diff-drive/holonomic robots
      w_curve: 30.0                 # weight to enforce minimum_turning_radius
      w_dist: 0.0                   # weight to bind path to original as optional replacement for cost weight
      w_smooth: 2000000.0           # weight to maximize smoothness of path
      w_cost: 0.015                 # weight to steer robot away from collision and cost

      # Parameters used to improve obstacle avoidance near cusps (forward/reverse movement changes)
      # See the [docs page](https://navigation.ros.org/configuration/packages/configuring-constrained-smoother) for further clarification
      w_cost_cusp_multiplier: 3.0   # option to have higher weight during forward/reverse direction change which is often accompanied with dangerous rotations
      cusp_zone_length: 2.5         # length of the section around cusp in which nodes use w_cost_cusp_multiplier (w_cost rises gradually inside the zone towards the cusp point, whose costmap weight equals w_cost*w_cost_cusp_multiplier)

      # Points in robot frame to grab costmap values from. Format: [x1, y1, weight1, x2, y2, weight2, ...]
      # IMPORTANT: Requires much higher number of iterations to actually improve the path. Uncomment only if you really need it (highly elongated/asymmetric robots)
      # See the [docs page](https://navigation.ros.org/configuration/packages/configuring-constrained-smoother) for further clarification
      # cost_check_points: [-0.185, 0.0, 1.0]

      optimizer:
        max_iterations: 70            # max iterations of smoother
        debug_optimizer: false        # print debug info
        gradient_tol: 5e3
        fn_tol: 1.0e-15
        param_tol: 1.0e-20
```

Note: Smoothing paths which contain multiple subsequent poses at one point (e.g. in-place rotations from Smac lattice planners) is currently not supported

Note: Constrained Smoother is recommended to be used on a path with a bounded length. TruncatePathLocal BT Node can be used for extracting a relevant path section around robot (in combination with DistanceController to achieve periodicity)

# Constrained Smoother（约束平滑器）

一个为 `nav2_smoother` 提供的平滑插件，基于 `nav2_smac_planner` 中已弃用的平滑器（作者 Steve Macenski），并由 RoboTech Vision 将其改造成可用状态。适用于需要将全局路径推离障碍物和/或用于 Reeds-Shepp 运动模型的场景。

文档见 navigation.ros.org: https://navigation.ros.org/configuration/packages/configuring-constrained-smoother.html

示例配置（完整 launch 配置请参见本仓库的 indoor_navigation 包）：

```yaml
smoother_server:
  ros__parameters:
    use_sim_time: True
    smoother_plugins: ["SmoothPath"]

    SmoothPath:
      plugin: "nav2_constrained_smoother/ConstrainedSmoother"
      reversing_enabled: true       # 是否检测前进/倒退方向变化及 cusp（鞍点）。对于没有方向信息的路径应设为 false
      path_downsampling_factor: 3   # 每隔 n 个节点取一个，用于加速
      path_upsampling_factor: 1     # 0 - 路径保持下采样粒度，1 - 使用三次贝塞尔上采样回原始粒度，2... - 更高的上采样倍数
      keep_start_orientation: true  # 是否防止起点朝向被平滑修改
      keep_goal_orientation: true   # 是否防止目标点朝向被平滑修改
      minimum_turning_radius: 0.40  # 机器人可执行的最小转弯半径。对差分驱动/全向机器人可设为 0.0（或把 w_curve 设为 0.0，效果相同）
      w_curve: 30.0                 # 强制最小转弯半径的权重
      w_dist: 0.0                   # 将路径约束为原始路径的权重（可作为替代 cost 权重）
      w_smooth: 2000000.0           # 最大化路径平滑性的权重
      w_cost: 0.015                 # 引导机器人远离碰撞和高代价区域的权重

      # 在 cusp（前进/倒退方向变化）附近提高避障能力的参数
      # 详见文档页面
      w_cost_cusp_multiplier: 3.0   # 在方向变化处增大权重的倍数（常伴随危险旋转）
      cusp_zone_length: 2.5         # 在 cusp 周围的区间长度，在区间内节点逐渐使用 w_cost_cusp_multiplier（在 cusp 点代价权重等于 w_cost * w_cost_cusp_multiplier）

      # 在机器人坐标系下用于读取 costmap 值的点。格式：[x1, y1, weight1, x2, y2, weight2, ...]
      # 重要：启用后需要大幅增加迭代次数才能对路径改进有效。仅在确实需要（长条/不对称机器人）时取消注释
      # 详见文档页面
      # cost_check_points: [-0.185, 0.0, 1.0]

      optimizer:
        max_iterations: 70            # 平滑器最大迭代次数
        debug_optimizer: false        # 是否打印调试信息
        gradient_tol: 5e3
        fn_tol: 1.0e-15
        param_tol: 1.0e-20
```

注意：
- 目前不支持对包含多个连续位于同一点姿态的路径（例如来自 Smac 格栅规划器的原地旋转）进行平滑处理。
- 建议对有界长度的路径使用 Constrained Smoother。可结合 TruncatePathLocal BT 节点提取机器人周围的相关路径段（与 DistanceController 配合以实现周期性）。