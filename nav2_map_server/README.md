# Map Server

The `Map Server` provides maps to the rest of the Nav2 system using both topic and
service interfaces. Map server will expose maps on the node bringup, but can also change maps using a `load_map` service during run-time, as well as save maps using a `save_map` server.

See its [Configuration Guide Page](https://navigation.ros.org/configuration/packages/configuring-map-server.html) for additional parameter descriptions.

### Architecture

In contrast to the ROS1 navigation map server, the nav2 map server will support a variety
of map types, and thus some aspects of the original code have been refactored to support
this new extensible framework.

Currently map server divides into tree parts:

- `map_server`
- `map_saver`
- `map_io` library

`map_server` is responsible for loading the map from a file through command-line interface
or by using service requests.

`map_saver` saves the map into a file. Like `map_server`, it has an ability to save the map from
command-line or by calling a service.

`map_io` - is a map input-output library. The library is designed to be an object-independent
in order to allow easily save/load map from external code just by calling necessary function.
This library is also used by `map_loader` and `map_saver` to work. Currently it contains
OccupancyGrid saving/loading functions moved from the rest part of map server code.
It is designed to be replaceable for a new IO library (e.g. for library with new map encoding
method or any other library supporting costmaps, multifloor maps, etc...).

### CLI-usage

#### Map Server

The `Map Server` is a composable ROS2 node. By default, there is a `map_server` executable that
instances one of these nodes, but it is possible to compose multiple map server nodes into
a single process, if desired.

The command line for the map server executable is slightly different that it was with ROS1.
With ROS1, one invoked the map server and passing the map YAML filename, like this:

```
$ map_server map.yaml
```

Where the YAML file specified contained the various map metadata, such as:

```
image: testmap.png
resolution: 0.1
origin: [2.0, 3.0, 1.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

The Nav2 software retains the map YAML file format from Nav1, but uses the ROS2 parameter
mechanism to get the name of the YAML file to use. This effectively introduces a
level of indirection to get the map yaml filename. For example, for a node named 'map_server',
the parameter file would look like this:

```
# map_server_params.yaml
map_server:
    ros__parameters:
        yaml_filename: "map.yaml"
```

One can invoke the map service executable directly, passing the params file on the command line,
like this:

```
$ map_server __params:=map_server_params.yaml
```

There is also possibility of having multiple map server nodes in a single process, where the parameters file would separate the parameters by node name, like this:

```
# combined_params.yaml
map_server1:
    ros__parameters:
        yaml_filename: "some_map.yaml"

map_server2:
    ros__parameters:
        yaml_filename: "another_map.yaml"
```

Then, one would invoke this process with the params file that contains the parameters for both nodes:

```
$ process_with_multiple_map_servers __params:=combined_params.yaml
```


The parameter for the initial map (yaml_filename) has to be set, but an empty string can be used if no initial map should be loaded. In this case, no map is loaded during
on_configure or published during on_activate. The _load_map_-service should the be used to load and publish a map.


#### Map Saver

Like in ROS1 `map_saver` could be used as CLI-executable. It was renamed to `map_saver_cli`
and could be invoked by following command:

```
$ ros2 run nav2_map_server map_saver_cli [arguments] [--ros-args ROS remapping args]
```

## Currently Supported Map Types

- Occupancy grid (nav_msgs/msg/OccupancyGrid)

## MapIO library

`MapIO` library contains following API functions declared in `map_io.hpp` to work with
OccupancyGrid maps:

- loadMapYaml(): Load and parse the given YAML file
- loadMapFromFile(): Load the image from map file and generate an OccupancyGrid
- loadMapFromYaml(): Load the map YAML, image from map file and generate an OccupancyGrid
- saveMapToFile(): Write OccupancyGrid map to file

## Services

As in ROS navigation, the `map_server` node provides a "map" service to get the map. See the nav_msgs/srv/GetMap.srv file for details.

NEW in ROS2 Eloquent, `map_server` also now provides a "load_map" service and `map_saver` -
a "save_map" service. See nav2_msgs/srv/LoadMap.srv and nav2_msgs/srv/SaveMap.srv for details.

For using these services `map_server`/`map_saver` should be launched as a continuously running
`nav2::LifecycleNode` node. In addition to the CLI, `Map Saver` has a functionality of server
handling incoming services. To run `Map Saver` in a server mode
`nav2_map_server/launch/map_saver_server.launch.py` launch-file could be used.

Service usage examples:

```
$ ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"
$ ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: my_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```


# Map Server（中文翻译）

`Map Server` 为 Nav2 系统的其余部分提供地图，支持话题和服务接口。Map server 在节点启动时会发布地图，也可以在运行时通过 `load_map` 服务更换地图，或通过 `save_map` 服务保存地图。

更多参数说明见官方配置指南页面：https://navigation.ros.org/configuration/packages/configuring-map-server.html

### 架构

与 ROS1 的 map server 不同，Nav2 map server 支持多种地图类型，因此对原有代码做了重构以提供可扩展框架。

当前 map server 分为三部分：
- `map_server`
- `map_saver`
- `map_io` 库

- `map_server` 负责通过命令行或服务请求加载地图文件。
- `map_saver` 将地图保存到文件，可通过命令行或服务调用保存。
- `map_io` 是一个地图 I/O 库，提供与 OccupancyGrid 相关的保存/加载函数，设计为与对象无关，便于从外部代码调用并替换实现（例如支持不同编码或多楼层地图的库）。

### CLI 用法

#### Map Server

`Map Server` 是可组合（composable）的 ROS2 节点。默认有一个 `map_server` 可执行文件用于实例化节点，也可以将多个 map server 以组件形式组合到同一进程中。

与 ROS1 不同，Nav2 使用 ROS2 参数机制来获取 YAML 文件名，示例：

ROS1：
```
$ map_server map.yaml
```

YAML 示例（map 元数据）：
```
image: testmap.png
resolution: 0.1
origin: [2.0, 3.0, 1.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

在 Nav2 中，参数文件示例（节点名为 `map_server`）：
```yaml
# map_server_params.yaml
map_server:
    ros__parameters:
        yaml_filename: "map.yaml"
```

可用命令行启动：
```
$ map_server __params:=map_server_params.yaml
```

若在同一进程中运行多个 map server，参数文件可按节点名区分，例如：
```yaml
# combined_params.yaml
map_server1:
    ros__parameters:
        yaml_filename: "some_map.yaml"

map_server2:
    ros__parameters:
        yaml_filename: "another_map.yaml"
```
然后：
```
$ process_with_multiple_map_servers __params:=combined_params.yaml
```

初始地图参数 `yaml_filename` 必须设置；若设置为空字符串，则在 `on_configure` 不加载任何地图，也不会在 `on_activate` 发布地图，此时应使用 `_load_map` 服务加载并发布地图。

#### Map Saver

`map_saver` 在 ROS2 中重命名为 `map_saver_cli`，可通过：
```
$ ros2 run nav2_map_server map_saver_cli [arguments] [--ros-args ROS remapping args]
```
运行。

## 当前支持的地图类型

- Occupancy grid（`nav_msgs/msg/OccupancyGrid`）

## MapIO 库

`MapIO` 提供以下 API（在 `map_io.hpp` 中声明），用于 OccupancyGrid 地图操作：
- loadMapYaml(): 加载并解析给定 YAML 文件
- loadMapFromFile(): 从地图文件加载图像并生成 OccupancyGrid
- loadMapFromYaml(): 从 YAML 加载并生成 OccupancyGrid
- saveMapToFile(): 将 OccupancyGrid 写入文件

## 服务

- 提供 `map` 服务以获取地图（参见 `nav_msgs/srv/GetMap.srv`）。
- ROS2 中新增 `load_map`（`nav2_msgs/srv/LoadMap.srv`）和 `save_map`（`nav2_msgs/srv/SaveMap.srv`）服务。
- 要使用这些服务，`map_server`/`map_saver` 应作为持续运行的 `nav2::LifecycleNode` 启动。示例命令：
```
$ ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: /ros/maps/map.yaml}"
$ ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: my_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```

以上为 `nav2_map_server/README.md` 的中文翻译。

Similar code found with 1 license type