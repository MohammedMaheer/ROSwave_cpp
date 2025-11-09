// Minimal structured JSON-lines logger implementation
#include "logging.hpp"

#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace ros2_dashboard {

void StructuredLogger::init(const std::string &path, std::size_t maxBytes) {
    auto &inst = StructuredLogger::instance();
    std::lock_guard<std::mutex> lk(inst.mtx_);
    inst.path_ = path;
    inst.maxBytes_ = maxBytes;
    if (inst.out_.is_open()) inst.out_.close();
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    inst.out_.open(path, std::ios::app);
}

StructuredLogger &StructuredLogger::instance() {
    static StructuredLogger inst;
    return inst;
}

std::string StructuredLogger::levelToString(Level l) const {
    switch (l) {
        case Level::Debug: return "debug";
        case Level::Info: return "info";
        case Level::Warning: return "warning";
        case Level::Error: return "error";
    }
    return "info";
}

void StructuredLogger::rotateIfNeeded() {
    if (path_.empty()) return;
    try {
        if (!std::filesystem::exists(path_)) return;
        auto sz = std::filesystem::file_size(path_);
        if (sz < maxBytes_) return;
        // close current and rotate
        if (out_.is_open()) out_.close();
        // timestamped backup
        auto t = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(t);
        std::tm tm = *std::localtime(&tt);
        std::ostringstream ss;
        ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
        std::string backup = path_ + "." + ss.str();
        std::filesystem::rename(path_, backup);
        out_.open(path_, std::ios::app);
    } catch (...) {
        // best-effort: ignore rotation errors
    }
}

void StructuredLogger::log(Level level, const std::string &message, const nlohmann::json &metadata) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!out_.is_open()) {
        if (!path_.empty()) {
            out_.open(path_, std::ios::app);
        }
    }
    rotateIfNeeded();

    nlohmann::json j;
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    j["ts_ms"] = ms;
    j["level"] = levelToString(level);
    j["msg"] = message;
    if (!metadata.is_null()) j["meta"] = metadata;

    try {
        out_ << j.dump() << '\n';
        out_.flush();
    } catch (...) {
        // swallow
    }
}

void StructuredLogger::debug(const std::string &message, const nlohmann::json &metadata) { log(Level::Debug, message, metadata); }
void StructuredLogger::info(const std::string &message, const nlohmann::json &metadata) { log(Level::Info, message, metadata); }
void StructuredLogger::warn(const std::string &message, const nlohmann::json &metadata) { log(Level::Warning, message, metadata); }
void StructuredLogger::error(const std::string &message, const nlohmann::json &metadata) { log(Level::Error, message, metadata); }

} // namespace ros2_dashboard
