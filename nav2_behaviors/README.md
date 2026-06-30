# Behaviors

The `nav2_behaviors` package implements a task server for executing behaviors.

The package defines:
- A `TimedBehavior` template which is used as a base class to implement specific timed behavior action server - but not required.
- The  `Backup`, `DriveOnHeading`, `Spin` and `Wait` behaviors.

The only required class a behavior must derive from is the `nav2_core/behavior.hpp` class, which implements the pluginlib interface the behavior server will use to dynamically load your behavior. The `nav2_behaviors/timed_behavior.hpp` derives from this class and implements a generic action server for a timed behavior behavior (e.g. calls an implmentation function on a regular time interval to compute a value) but **this is not required** if it is not helpful. A behavior does not even need to be an action if you do not wish, it may be a service or other interface. However, most motion and behavior primitives are probably long-running and make sense to be modeled as actions, so the provided `timed_behavior.hpp` helps in managing the complexity to simplify new behavior development, described more below.

The value of the centralized behavior server is to **share resources** amongst several behaviors that would otherwise be independent nodes. Subscriptions to TF, costmaps, and more can be quite heavy and add non-trivial compute costs to a robot system. By combining these independent behaviors into a single server, they may share these resources while retaining complete independence in execution and interface.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-behavior-server.html) for additional parameter descriptions and a [tutorial about writing behaviors](https://navigation.ros.org/plugin_tutorials/docs/writing_new_behavior_plugin.html).

See the [Navigation Plugin list](https://navigation.ros.org/plugins/index.html) for a list of the currently known and available planner plugins.

# 行为

`nav2_behaviors` 包实现了一个用于执行行为的任务服务器。

该包定义了：
- 一个 `TimedBehavior` 模板，作为实现特定定时行为动作服务器的基类（可选，非必须）。
- `Backup`、`DriveOnHeading`、`Spin` 和 `Wait` 这些行为。

行为必须继承的唯一必需类是 `nav2_core/behavior.hpp`，该类实现了行为服务器用来动态加载行为的 pluginlib 接口。`nav2_behaviors/timed_behavior.hpp` 派生自该类，并为定时行为实现了一个通用的 action server（例如以固定时间间隔调用实现函数来计算值），但如果对你的需求没有帮助则**非必须**使用。行为也不一定要以 action 的形式实现，它可以是 service 或其它接口。不过大多数运动与行为原语通常是长时运行的，适合用 action 建模，因此提供的 `timed_behavior.hpp` 有助于管理复杂性，简化新行为的开发（下面有更多说明）。

集中式行为服务器的价值在于可以在多个原本独立的行为之间**共享资源**。订阅 TF、代价地图等会消耗较多资源并给机器人系统带来显著计算开销。通过把这些独立行为合并到单一服务器中，它们可以共享这些资源，同时在执行和接口上保持完全独立。

更多参数说明见其 [配置指南](https://navigation.ros.org/configuration/packages/configuring-behavior-server.html)，关于如何编写行为的教程见 [行为插件编写教程](https://navigation.ros.org/plugin_tutorials/docs/writing_new_behavior_plugin.html)。

当前已知和可用的规划器插件列表见 [Navigation Plugin 列表](https://navigation.ros.org/plugins/index.html)。