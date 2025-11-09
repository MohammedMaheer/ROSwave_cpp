#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <thread>
#include <mutex>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <nlohmann/json.hpp>

// Minimal test harness for logging functionality
namespace fs = std::filesystem;
using json = nlohmann::json;

// Stub implementation of logging for testing (matches include/logging.hpp)
namespace ros2_dashboard {

class StructuredLogger {
public:
    enum class Level { Debug, Info, Warning, Error };

    static void init(const std::string &path, std::size_t maxBytes = 10 * 1024 * 1024) {
        auto &inst = instance();
        std::lock_guard<std::mutex> lk(inst.mtx_);
        inst.path_ = path;
        inst.maxBytes_ = maxBytes;
        if (inst.out_.is_open()) inst.out_.close();
        fs::create_directories(fs::path(path).parent_path());
        inst.out_.open(path, std::ios::app);
    }

    static StructuredLogger &instance() {
        static StructuredLogger inst;
        return inst;
    }

    void log(Level level, const std::string &message, const json &metadata = {}) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!out_.is_open()) {
            if (!path_.empty()) {
                out_.open(path_, std::ios::app);
            }
        }
        rotateIfNeeded();

        json j;
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

    void debug(const std::string &message, const json &metadata = {}) { log(Level::Debug, message, metadata); }
    void info(const std::string &message, const json &metadata = {}) { log(Level::Info, message, metadata); }
    void warn(const std::string &message, const json &metadata = {}) { log(Level::Warning, message, metadata); }
    void error(const std::string &message, const json &metadata = {}) { log(Level::Error, message, metadata); }

private:
    StructuredLogger() = default;

    void rotateIfNeeded() {
        if (path_.empty()) return;
        try {
            if (!fs::exists(path_)) return;
            auto sz = fs::file_size(path_);
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
            fs::rename(path_, backup);
            out_.open(path_, std::ios::app);
        } catch (...) {
            // best-effort: ignore rotation errors
        }
    }

    std::string levelToString(Level l) const {
        switch (l) {
            case Level::Debug: return "debug";
            case Level::Info: return "info";
            case Level::Warning: return "warning";
            case Level::Error: return "error";
        }
        return "info";
    }

    std::mutex mtx_;
    std::ofstream out_;
    std::string path_;
    std::size_t maxBytes_{10 * 1024 * 1024};
};

} // namespace ros2_dashboard

// ===== Test Suite =====

bool test_logging_basic_write() {
    // Clean up any prior test file
    std::string test_log = "/tmp/test_logging.log";
    if (fs::exists(test_log)) fs::remove(test_log);

    // Initialize and write a few log entries
    ros2_dashboard::StructuredLogger::init(test_log, 1024 * 1024);
    auto &logger = ros2_dashboard::StructuredLogger::instance();

    logger.info("Test message 1");
    logger.debug("Debug entry", json{{"key", "value"}});
    logger.warn("Warning entry");
    logger.error("Error entry", json{{"code", 42}, {"reason", "test"}});

    // Read and verify
    std::ifstream in(test_log);
    std::string line;
    int count = 0;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        try {
            auto j = json::parse(line);
            if (!j.contains("ts_ms") || !j.contains("level") || !j.contains("msg")) {
                std::cerr << "  ✗ Malformed JSON line: " << line << '\n';
                return false;
            }
            count++;
        } catch (const std::exception &e) {
            std::cerr << "  ✗ JSON parse error: " << e.what() << '\n';
            return false;
        }
    }
    in.close();

    if (count != 4) {
        std::cerr << "  ✗ Expected 4 log lines, got " << count << '\n';
        return false;
    }

    fs::remove(test_log);
    std::cout << "  ✅ test_logging_basic_write: PASS\n";
    return true;
}

bool test_logging_json_levels() {
    std::string test_log = "/tmp/test_logging_levels.log";
    if (fs::exists(test_log)) fs::remove(test_log);

    ros2_dashboard::StructuredLogger::init(test_log, 1024 * 1024);
    auto &logger = ros2_dashboard::StructuredLogger::instance();

    logger.debug("debug msg");
    logger.info("info msg");
    logger.warn("warn msg");
    logger.error("error msg");

    std::ifstream in(test_log);
    std::string line;
    std::vector<std::string> levels;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        try {
            auto j = json::parse(line);
            levels.push_back(j["level"].get<std::string>());
        } catch (...) {
            std::cerr << "  ✗ Failed to parse JSON\n";
            return false;
        }
    }
    in.close();

    std::vector<std::string> expected{"debug", "info", "warning", "error"};
    if (levels != expected) {
        std::cerr << "  ✗ Levels mismatch. Expected [debug, info, warning, error]\n";
        return false;
    }

    fs::remove(test_log);
    std::cout << "  ✅ test_logging_json_levels: PASS\n";
    return true;
}

bool test_logging_metadata() {
    std::string test_log = "/tmp/test_logging_meta.log";
    if (fs::exists(test_log)) fs::remove(test_log);

    ros2_dashboard::StructuredLogger::init(test_log, 1024 * 1024);
    auto &logger = ros2_dashboard::StructuredLogger::instance();

    json meta{{"component", "test"}, {"value", 123}};
    logger.info("Test with metadata", meta);

    std::ifstream in(test_log);
    std::string line;
    bool found_meta = false;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        try {
            auto j = json::parse(line);
            if (j.contains("meta")) {
                auto m = j["meta"];
                if (m["component"] == "test" && m["value"] == 123) {
                    found_meta = true;
                }
            }
        } catch (...) {
        }
    }
    in.close();

    if (!found_meta) {
        std::cerr << "  ✗ Metadata not found or incorrect\n";
        return false;
    }

    fs::remove(test_log);
    std::cout << "  ✅ test_logging_metadata: PASS\n";
    return true;
}

bool test_logging_thread_safety() {
    std::string test_log = "/tmp/test_logging_thread.log";
    if (fs::exists(test_log)) fs::remove(test_log);

    ros2_dashboard::StructuredLogger::init(test_log, 1024 * 1024);
    auto &logger = ros2_dashboard::StructuredLogger::instance();

    // Write from multiple threads
    std::thread t1([&]() {
        for (int i = 0; i < 10; ++i) {
            logger.info("Thread 1 msg " + std::to_string(i));
        }
    });
    std::thread t2([&]() {
        for (int i = 0; i < 10; ++i) {
            logger.info("Thread 2 msg " + std::to_string(i));
        }
    });
    t1.join();
    t2.join();

    std::ifstream in(test_log);
    std::string line;
    int count = 0;
    while (std::getline(in, line)) {
        if (!line.empty()) count++;
    }
    in.close();

    if (count != 20) {
        std::cerr << "  ✗ Expected 20 lines from threads, got " << count << '\n';
        return false;
    }

    fs::remove(test_log);
    std::cout << "  ✅ test_logging_thread_safety: PASS\n";
    return true;
}

bool test_logging_rotation() {
    std::string test_log = "/tmp/test_logging_rotate.log";
    // Clean up any prior files
    for (const auto &entry : fs::directory_iterator("/tmp")) {
        if (entry.path().filename().string().find("test_logging_rotate") == 0) {
            fs::remove(entry.path());
        }
    }

    // Create logger with very small max size to trigger rotation
    ros2_dashboard::StructuredLogger::init(test_log, 200);  // 200 bytes max
    auto &logger = ros2_dashboard::StructuredLogger::instance();

    // Write enough to trigger rotation
    for (int i = 0; i < 10; ++i) {
        logger.info("Log entry number " + std::to_string(i) + " with some padding to increase size");
    }

    // Check for rotated files
    int rotated_count = 0;
    for (const auto &entry : fs::directory_iterator("/tmp")) {
        if (entry.path().filename().string().find("test_logging_rotate") == 0) {
            rotated_count++;
        }
    }

    // We should have at least 2 files (original + 1 rotated)
    if (rotated_count < 2) {
        std::cerr << "  ✗ Expected at least 2 files after rotation, got " << rotated_count << '\n';
        return false;
    }

    // Clean up
    for (const auto &entry : fs::directory_iterator("/tmp")) {
        if (entry.path().filename().string().find("test_logging_rotate") == 0) {
            fs::remove(entry.path());
        }
    }

    std::cout << "  ✅ test_logging_rotation: PASS\n";
    return true;
}

int main() {
    std::cout << "\n=== Logging Unit Test Suite ===\n\n";

    bool all_pass = true;
    all_pass &= test_logging_basic_write();
    all_pass &= test_logging_json_levels();
    all_pass &= test_logging_metadata();
    all_pass &= test_logging_thread_safety();
    all_pass &= test_logging_rotation();

    std::cout << "\n";
    if (all_pass) {
        std::cout << "✅ ALL LOGGING TESTS PASSED\n\n";
        return 0;
    } else {
        std::cout << "❌ SOME LOGGING TESTS FAILED\n\n";
        return 1;
    }
}
