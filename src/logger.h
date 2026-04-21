#pragma once

#include <spdlog/spdlog.h>
#include <string>

namespace Logger {

// Initialize the global logger with a stdout color sink and a rotating file sink.
// Call this once at application startup before any logging occurs.
// logFile: path to the log file (parent directories are created automatically)
// maxFileSize: max size in bytes before rotation (default 5 MB)
// maxFiles: number of rotated files to keep (default 3)
void init(const std::string& logFile = "logs/app.log",
          std::size_t maxFileSize = 5 * 1024 * 1024,
          std::size_t maxFiles = 3);

} // namespace Logger
