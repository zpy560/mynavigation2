#pragma once
#include <rcutils/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE

#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include "level_padded_flag.hpp"

#ifndef PACKAGE_NAME
#define PACKAGE_NAME "spdlog_wrapper"
#endif

// 日志配置结构
struct SpdlogConfig
{
    std::string log_dir = "/home/byd/logs";     // 日志目录
    size_t max_file_size_mb = 10 * 1024 * 1024; // 单个文件最大字节数 (MB)
    size_t max_files = 5;                       // 保留的历史文件个数
    std::string console_level = "trace";        // 控制台输出等级 trace/debug/info/warn/error/critical/off
    std::string file_level = "trace";           // 文件输出等级
    int flush_interval_seconds = 0;             // 日志刷新间隔
    bool append_ros_timestamp = false;          // 是否在 ROS 日志后附加 ROS 时间戳
    bool raw_ros_timestamp = false;             // true: 输出原始纳秒整数；false: 输出格式化时间字符串
};

class SpdlogWrapper
{
public:

    // 初始化日志系统（线程安全，只执行一次）
    static void init(const std::string &module,
                     const std::string &node_name);

    // 关闭日志系统，等待后台线程完成
    static void shutdown();

    static std::string short_file_name(const char *path);

private:
    // 从环境变量加载配置
    static SpdlogConfig load_config_from_env();

    // 从 YAML 文件加载指定模块的配置，若模块未定义则使用 default 节
    static SpdlogConfig load_from_yaml(const std::string &node_name);

    // 将 rcutils 时间戳格式化为人类可读字符串
    static std::string format_ros_time(rcutils_time_point_value_t timestamp);

    // 将 rcutils 严重等级映射为 spdlog 等级
    static spdlog::level::level_enum level_to_spdlog(int severity);

    // 自定义日志输出处理器（劫持 ROS2 日志流）
    static void output_handler(const rcutils_log_location_t *location,
                               int severity,
                               const char * /*name*/,
                               rcutils_time_point_value_t timestamp,
                               const char *format,
                               va_list *args);

    static inline std::shared_ptr<spdlog::async_logger> g_logger;
    static inline SpdlogConfig g_config = {}; // 或 = {}
    static inline spdlog::level::level_enum g_min_level;
};

#define LOG_TRACE(fmt, ...)                                \
    SPDLOG_TRACE("[{} {}:{}] " fmt,                        \
                 SpdlogWrapper::short_file_name(__FILE__), \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define LOG_DEBUG(fmt, ...)                                \
    SPDLOG_DEBUG("[{} {}:{}] " fmt,                        \
                 SpdlogWrapper::short_file_name(__FILE__), \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define LOG_INFO(fmt, ...)                                \
    SPDLOG_INFO("[{} {}:{}] " fmt,                        \
                SpdlogWrapper::short_file_name(__FILE__), \
                __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define LOG_WARN(fmt, ...)                                \
    SPDLOG_WARN("[{} {}:{}] " fmt,                        \
                SpdlogWrapper::short_file_name(__FILE__), \
                __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define LOG_ERROR(fmt, ...)                                \
    SPDLOG_ERROR("[{} {}:{}] " fmt,                        \
                 SpdlogWrapper::short_file_name(__FILE__), \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__)

#define LOG_CRITICAL(fmt, ...)                                \
    SPDLOG_CRITICAL("[{} {}:{}] " fmt,                        \
                    SpdlogWrapper::short_file_name(__FILE__), \
                    __FUNCTION__, __LINE__, ##__VA_ARGS__)
