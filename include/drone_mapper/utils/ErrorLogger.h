#pragma once

#include <filesystem>
#include <string_view>

namespace drone_mapper {

// Writes timestamped error entries to a file on disk.
// Each call to log() opens the file in append mode, writes one line, flushes, and closes —
// guaranteeing the entry is durable even if the process crashes immediately after.
// The log file and its parent directories are created lazily on the first log() call.
class ErrorLogger {
public:
    // Stores the target log file path. The file is not created until the first log() call.
    explicit ErrorLogger(std::filesystem::path log_file);

    // Appends one line in the format: [<UTC timestamp>] ERROR <code>: <message>
    // Thread-unsafe: external synchronisation is required if called from multiple threads.
    void log(std::string_view code, std::string_view message);

private:
    std::filesystem::path log_file_;
};

} // namespace drone_mapper
