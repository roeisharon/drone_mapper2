#include <drone_mapper/utils/ErrorLogger.h>

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace drone_mapper {

namespace {

// Formats the current wall-clock time as an ISO 8601 UTC string, e.g. "2026-06-10T14:03:22Z".
std::string utcTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_utc{};
#ifdef _WIN32
    gmtime_s(&tm_utc, &t);
#else
    gmtime_r(&t, &tm_utc);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

} // namespace

ErrorLogger::ErrorLogger(std::filesystem::path log_file)
    : log_file_(std::move(log_file)) {}

void ErrorLogger::log(std::string_view code, std::string_view message) {
    // Create the parent directory tree if it does not already exist, so callers can
    // pass nested paths like "output_results/run_1/errors.log" without pre-creating them.
    if (log_file_.has_parent_path()) {
        std::filesystem::create_directories(log_file_.parent_path());
    }

    // Append mode: previous entries are never overwritten.
    std::ofstream out(log_file_, std::ios::app);
    if (!out.is_open()) {
        throw std::runtime_error("ErrorLogger: cannot open log file: " + log_file_.string());
    }

    out << "[" << utcTimestamp() << "] ERROR " << code << ": " << message << "\n";

    // Flush before the stream closes so the data reaches disk even if the process is
    // killed right after this call. Opening per call (rather than holding the stream) is
    // intentional — it keeps every entry durable.
    out.flush();
}

} // namespace drone_mapper
