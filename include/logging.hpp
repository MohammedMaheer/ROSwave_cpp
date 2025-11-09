// Minimal structured JSON-lines logger with rotation
#pragma once

#include <string>
#include <fstream>
#include <mutex>
#include <memory>
#include <nlohmann/json.hpp>

namespace ros2_dashboard {

class StructuredLogger {
public:
    enum class Level { Debug, Info, Warning, Error };

    // Initialize logger with path and max size in bytes for rotation
    static void init(const std::string &path, std::size_t maxBytes = 10 * 1024 * 1024);
    static StructuredLogger &instance();

    void log(Level level, const std::string &message, const nlohmann::json &metadata = {});

    // Convenience helpers
    void debug(const std::string &message, const nlohmann::json &metadata = {});
    void info(const std::string &message, const nlohmann::json &metadata = {});
    void warn(const std::string &message, const nlohmann::json &metadata = {});
    void error(const std::string &message, const nlohmann::json &metadata = {});

private:
    StructuredLogger() = default;
    void rotateIfNeeded();
    std::string levelToString(Level l) const;

    std::mutex mtx_;
    std::ofstream out_;
    std::string path_;
    std::size_t maxBytes_{10 * 1024 * 1024};
};

} // namespace ros2_dashboard
