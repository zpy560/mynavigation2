#pragma once

#include <spdlog/pattern_formatter.h>
#include <memory>

class level_padded_flag : public spdlog::custom_flag_formatter
{
public:
    void format(const spdlog::details::log_msg& msg,
                const std::tm&,
                spdlog::memory_buf_t& dest) override
    {
        auto level_str = spdlog::level::to_string_view(msg.level);
        dest.append(level_str.data(), level_str.data() + level_str.size());

        // 固定宽度为 8 个字符，不足右侧补空格
        constexpr size_t kWidth = 8;
        if (level_str.size() < kWidth)
            for (size_t i = 0; i < kWidth - level_str.size(); ++i)
            {
                dest.push_back(' ');
            }
    }

    std::unique_ptr<custom_flag_formatter> clone() const override
    {
        return std::make_unique<level_padded_flag>();
    }
};