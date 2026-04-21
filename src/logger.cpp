#include "logger.h"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <filesystem>

namespace Logger {

void init(const std::string& logFile, std::size_t maxFileSize, std::size_t maxFiles) {
    // Ensure the log directory exists
    std::filesystem::path logPath(logFile);
    if (logPath.has_parent_path()) {
        std::filesystem::create_directories(logPath.parent_path());
    }

    auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    consoleSink->set_level(spdlog::level::debug);

    auto fileSink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        logFile, maxFileSize, maxFiles);
    fileSink->set_level(spdlog::level::debug);

    auto logger = std::make_shared<spdlog::logger>(
        "app", spdlog::sinks_init_list{consoleSink, fileSink});

    logger->set_level(spdlog::level::debug);
    logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%t] %v");

    spdlog::set_default_logger(logger);
    spdlog::flush_on(spdlog::level::warn);
}

} // namespace Logger
