#include "spdlog_wrapper.hpp"

void SpdlogWrapper::init(const std::string &module, const std::string &node_name)
{
    static std::once_flag once;
    std::call_once(once, [&]()
                   {
        g_config = load_from_yaml(node_name);

        // 日志目录
        std::string dir = g_config.log_dir;
        std::filesystem::create_directories(dir + "/" + module);
        std::string file = dir + "/" + module + "/" + node_name + ".log";

        // 队列大小 8192，1 个后台线程
        spdlog::init_thread_pool(8192, 1);

        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_color_mode(spdlog::color_mode::always);
        // 自定义颜色
        console_sink->set_color(spdlog::level::info, "\033[37m");   // 白色

        auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            file, g_config.max_file_size_mb, g_config.max_files);

        std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};

        g_logger = std::make_shared<spdlog::async_logger>(
            node_name,
            sinks.begin(),
            sinks.end(),
            spdlog::thread_pool(),
            spdlog::async_overflow_policy::overrun_oldest); // 满了丢旧日志
        spdlog::register_logger(g_logger); // 注册logger
        spdlog::set_default_logger(g_logger);

        // 日志输出级别
        console_sink->set_level(spdlog::level::from_str(g_config.console_level));
        file_sink->set_level(spdlog::level::from_str(g_config.file_level));
        g_min_level = std::min(console_sink->level(), file_sink->level());

        if (g_config.flush_interval_seconds > 0)
        {
            spdlog::flush_every(std::chrono::seconds(g_config.flush_interval_seconds));
        }
        // 错误立马刷新
        g_logger->flush_on(spdlog::level::err);

        console_sink->set_pattern("%^[%Y-%m-%d %H:%M:%S.%e] [%l] [tid:%t] %v%$");
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [tid:%t] %v");

        // // 构建 pattern 字符串（level改用自定义 %L, 方便对齐）
        // // 为每个 sink 创建独立的 formatter
        // auto console_formatter = std::make_unique<spdlog::pattern_formatter>();
        // console_formatter->add_flag<level_padded_flag>('L');
        // console_formatter->set_pattern("%^[%Y-%m-%d %H:%M:%S.%e] [%L] [tid:%t] %v%$");
        // console_sink->set_formatter(std::move(console_formatter));

        // auto file_formatter = std::make_unique<spdlog::pattern_formatter>();
        // file_formatter->add_flag<level_padded_flag>('L');
        // file_formatter->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%L] [tid:%t] %v");
        // file_sink->set_formatter(std::move(file_formatter));

        rcutils_logging_set_output_handler(&SpdlogWrapper::output_handler); });
}

void SpdlogWrapper::shutdown()
{
    g_logger.reset();
    spdlog::shutdown();
}

// ---------- 配置文件加载 ----------

SpdlogConfig SpdlogWrapper::load_from_yaml(const std::string &node_name)
{
    std::string yaml_path;
    try
    {
        yaml_path = ament_index_cpp::get_package_share_directory(PACKAGE_NAME) + "/config/log.yaml";
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error("获取共享目录失败: " + std::string(e.what()));
    }

    SpdlogConfig cfg;

    try
    {
        YAML::Node root = YAML::LoadFile(yaml_path);
        if (!root.IsMap())
        {
            std::cout << "YAML root is not a map" << std::endl;
            return cfg; // 格式不对，直接返回默认配置
        }

        // 2. 确定要读取的节点：优先节点，否则 default
        YAML::Node target;
        if (root[node_name])
        {
            target = root[node_name];
        }
        else if (root["default"])
        {
            target = root["default"];
        }
        else
        {
            return cfg; // 既无模块也无 default，直接返回默认配置
        }

        if (target["log_dir"])
            cfg.log_dir = target["log_dir"].as<std::string>();
        if (target["max_file_size_mb"])
            cfg.max_file_size_mb = target["max_file_size_mb"].as<size_t>() * 1024 * 1024;
        if (target["max_files"])
            cfg.max_files = target["max_files"].as<size_t>();
        if (target["console_level"])
            cfg.console_level = target["console_level"].as<std::string>();
        if (target["file_level"])
            cfg.file_level = target["file_level"].as<std::string>();
        if (target["flush_interval_seconds"])
            cfg.flush_interval_seconds = target["flush_interval_seconds"].as<int>();
        if (target["append_ros_timestamp"])
            cfg.append_ros_timestamp = target["append_ros_timestamp"].as<bool>();
        if (target["raw_ros_timestamp"])
            cfg.raw_ros_timestamp = target["raw_ros_timestamp"].as<bool>();
    }
    catch (const YAML::Exception &e)
    {
        std::cerr << "[SpdlogWrapper] YAML parse error: " << e.what()
                  << ". Using default config." << std::endl;
    }

    const SpdlogConfig env_cfg = load_config_from_env();
    if (std::getenv("SPDLOG_WRAPPER_LOG_DIR"))
        cfg.log_dir = env_cfg.log_dir;
    if (std::getenv("SPDLOG_WRAPPER_MAX_FILE_SIZE_MB"))
        cfg.max_file_size_mb = env_cfg.max_file_size_mb;
    if (std::getenv("SPDLOG_WRAPPER_MAX_FILES"))
        cfg.max_files = env_cfg.max_files;

    return cfg;
}

// ---------- 环境变量加载 ----------

SpdlogConfig SpdlogWrapper::load_config_from_env()
{
    SpdlogConfig cfg;

    // 日志目录：SPDLOG_WRAPPER_LOG_DIR
    if (const char *env = std::getenv("SPDLOG_WRAPPER_LOG_DIR"))
        cfg.log_dir = env;

    // 单文件最大大小（MB）：SPDLOG_WRAPPER_MAX_FILE_SIZE_MB
    if (const char *env = std::getenv("SPDLOG_WRAPPER_MAX_FILE_SIZE_MB"))
    {
        size_t mb = std::atoi(env);
        if (mb > 0)
            cfg.max_file_size_mb = mb * 1024 * 1024;
    }

    // 最大文件数量：SPDLOG_WRAPPER_MAX_FILES
    if (const char *env = std::getenv("SPDLOG_WRAPPER_MAX_FILES"))
    {
        int num = std::atoi(env);
        if (num > 0)
            cfg.max_files = static_cast<size_t>(num);
    }

    return cfg;
}

spdlog::level::level_enum SpdlogWrapper::level_to_spdlog(int severity)
{
    switch (severity)
    {
    case RCUTILS_LOG_SEVERITY_DEBUG:
        return spdlog::level::debug;
    case RCUTILS_LOG_SEVERITY_INFO:
        return spdlog::level::info;
    case RCUTILS_LOG_SEVERITY_WARN:
        return spdlog::level::warn;
    case RCUTILS_LOG_SEVERITY_ERROR:
        return spdlog::level::err;
    case RCUTILS_LOG_SEVERITY_FATAL:
        return spdlog::level::critical;
    default:
        return spdlog::level::info;
    }
}

std::string SpdlogWrapper::format_ros_time(rcutils_time_point_value_t timestamp)
{
    using namespace std::chrono;

    auto tp = time_point<system_clock, nanoseconds>(nanoseconds(timestamp));
    auto sec_tp = time_point_cast<seconds>(tp);
    auto ms = duration_cast<milliseconds>(tp - sec_tp).count(); // 毫秒 (0~999)

    std::time_t tt = system_clock::to_time_t(sec_tp);
    std::tm tm{};
    localtime_r(&tt, &tm);

    char buf[64];
    std::snprintf(buf, sizeof(buf),
                  "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec, ms);
    return buf;
}

std::string SpdlogWrapper::short_file_name(const char *path)
{
    const char *short_file = strrchr(path, '/');
    short_file = short_file ? short_file + 1 : path;
    return std::string(short_file);
}

void SpdlogWrapper::output_handler(const rcutils_log_location_t *location,
                                   int severity,
                                   const char * /*name*/,
                                   rcutils_time_point_value_t timestamp,
                                   const char *format,
                                   va_list *args)
{
    auto logger = g_logger;
    if (!logger)
        return;

    spdlog::level::level_enum level = level_to_spdlog(severity);
    // 提前检查等级，不满足则立即返回，避免无用格式化
    if (level < g_min_level)
    {
        return;
    }

    char msg[2048];
    std::vsnprintf(msg, sizeof(msg), format, *args);

    const char *file = "unknown";
    const char *func = "unknown";
    int line = 0;

    if (location)
    {
        if (location->file_name)
            file = location->file_name;
        if (location->function_name)
            func = location->function_name;
        line = location->line_number;
    }

    // 去路径
    const std::string short_file = short_file_name(file);

    std::string ros_ts_suffix;
    if (g_config.append_ros_timestamp)
    {
        if (g_config.raw_ros_timestamp)
        {
            ros_ts_suffix = " [ros_ts:" + std::to_string(timestamp) + "]";
        }
        else
        {
            ros_ts_suffix = " [ros_ts:" + format_ros_time(timestamp) + "]";
        }
    }

    logger->log(level, "[{} {}:{}]{} {}", short_file, func, line, ros_ts_suffix, msg);
}
