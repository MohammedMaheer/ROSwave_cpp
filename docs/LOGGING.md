# Structured Logging Feature

## Overview

The **Structured Logging** feature provides a lightweight, thread-safe JSON-lines logger for the ROS 2 Dashboard. It enables detailed diagnostic and operational event tracking with automatic file rotation.

## Architecture

### StructuredLogger (include/logging.hpp)

The `StructuredLogger` class is a singleton that provides:
- Thread-safe logging with `std::mutex`
- JSON-lines output format (one JSON object per line)
- Automatic file rotation when size limits are exceeded
- Multiple log levels (Debug, Info, Warning, Error)
- Metadata support for rich contextual information

### Key Methods

```cpp
// Static initialization
static void init(const std::string& path, 
                std::size_t maxBytes = 10 * 1024 * 1024);

// Get singleton instance
static StructuredLogger& instance();

// Log with explicit level and optional metadata
void log(Level level, const std::string& message, 
        const nlohmann::json& metadata = {});

// Convenience methods for each log level
void debug(const std::string& message, const nlohmann::json& metadata = {});
void info(const std::string& message, const nlohmann::json& metadata = {});
void warn(const std::string& message, const nlohmann::json& metadata = {});
void error(const std::string& message, const nlohmann::json& metadata = {});
```

## Usage

### Basic Initialization

```cpp
#include "logging.hpp"
using namespace ros2_dashboard;

// Initialize logger to write to file
StructuredLogger::init("/tmp/dashboard.log", 50 * 1024 * 1024);  // 50 MB limit

auto& logger = StructuredLogger::instance();
```

### Simple Logging

```cpp
// Log without metadata
logger.info("Dashboard started successfully");

// Log with metadata
nlohmann::json meta{
    {"version", "1.0.0"},
    {"mode", "production"}
};
logger.info("Configuration loaded", meta);
```

### Log Levels

```cpp
logger.debug("Subscribed to topic", {{"topic", "/sensor/lidar"}});
logger.info("Processing metrics", {{"count", 42}});
logger.warn("High CPU usage detected", {{"usage_percent", 87.5}});
logger.error("Failed to connect", {{"endpoint", "localhost:1883"}, {"error_code", -1}});
```

## Log Format

Each log entry is a JSON object on a single line:

```json
{"ts_ms": 1699539600123, "level": "info", "msg": "Dashboard started", "meta": {"version": "1.0.0"}}
{"ts_ms": 1699539601456, "level": "error", "msg": "Connection failed", "meta": {"code": 404}}
```

### JSON Fields

| Field | Type | Description |
|-------|------|-------------|
| `ts_ms` | integer | Timestamp in milliseconds since epoch |
| `level` | string | Log level: "debug", "info", "warning", "error" |
| `msg` | string | Log message text |
| `meta` | object | Optional metadata (omitted if not provided) |

## File Rotation

Automatic rotation occurs when the log file exceeds the configured size limit:

```cpp
// Initialize with 50 MB limit
StructuredLogger::init("/var/log/dashboard.log", 50 * 1024 * 1024);
```

When rotation occurs:
1. Current log file is closed
2. File is renamed with timestamp: `dashboard.log.20231111_145623`
3. New log file is opened for writing
4. Logging continues without interruption

### Rotation Example

```
/var/log/dashboard.log                    (current, active)
/var/log/dashboard.log.20231110_103015    (rotated)
/var/log/dashboard.log.20231109_203847    (rotated)
```

## Thread Safety

The logger is fully thread-safe for concurrent writes from multiple threads:

```cpp
std::thread t1([]{
    StructuredLogger::instance().info("Thread 1 message");
});
std::thread t2([]{
    StructuredLogger::instance().info("Thread 2 message");
});
t1.join();
t2.join();
```

All writes are serialized with `std::mutex` to prevent interleaving.

## Metadata Best Practices

### Common Metadata Fields

```cpp
// Topic subscription
logger.info("Subscribed", {
    {"entity_type", "topic"},
    {"name", "/lidar/points"},
    {"msg_type", "sensor_msgs/PointCloud2"},
    {"subscriber_id", 42}
});

// Performance metrics
logger.info("Processing stats", {
    {"component", "metrics_collector"},
    {"duration_ms", 1234},
    {"items_processed", 5000}
});

// Errors
logger.error("Export failed", {
    {"file_path", "/tmp/export.bag"},
    {"reason", "disk_full"},
    {"available_mb", 0}
});

// Session events
logger.info("Session saved", {
    {"session_name", "experiment_001"},
    {"duration_sec", 3600},
    {"topics_recorded", 12}
});
```

## Integration Points

### Initialization in main.cpp

```cpp
#include "logging.hpp"

int main(int argc, char** argv) {
    // Initialize logging early
    std::string log_path = std::filesystem::temp_directory_path() / "ros2_dashboard.log";
    ros2_dashboard::StructuredLogger::init(log_path);
    ros2_dashboard::StructuredLogger::instance().info("Dashboard startup");
    
    // Continue with GUI initialization
    QApplication app(argc, argv);
    // ...
}
```

### Session Manager Integration

```cpp
// In SessionManager::save_session()
auto& logger = ros2_dashboard::StructuredLogger::instance();
logger.info("Saving session", {
    {"name", session_name},
    {"topics_count", subscribed_topics.size()},
    {"timestamp", current_timestamp}
});
```

### Alert Manager Integration

```cpp
// In AlertManager::emit_alert()
auto& logger = ros2_dashboard::StructuredLogger::instance();
logger.warn("Alert triggered", {
    {"alert_type", alert.type},
    {"threshold", alert.threshold},
    {"current_value", current_value},
    {"metric", alert.metric_name}
});
```

### Metrics Collector Integration

```cpp
// In MetricsCollector::collect()
auto& logger = ros2_dashboard::StructuredLogger::instance();
logger.debug("Metrics snapshot", {
    {"cpu_percent", cpu_usage},
    {"memory_mb", memory_usage},
    {"network_mbps", network_throughput}
});
```

## Performance

- **Write latency**: < 1 ms per log entry (typical)
- **Memory footprint**: ~2 KB base + JSON allocation per entry
- **File I/O**: Buffered with explicit `flush()` for durability
- **Rotation overhead**: One-time cost when size limit reached

## Testing

### Unit Tests

Run the logging test suite:

```bash
cd build
./test_logging
```

**Test Coverage:**
- Basic write verification (4 entries)
- Log level correctly recorded
- Metadata preservation and retrieval
- Thread-safe concurrent writes (20 entries from 2 threads)
- File rotation triggering and naming
- Edge cases (empty messages, large metadata)

All tests validate both correct JSON format and correct behavior.

### Manual Testing

```bash
# Initialize and log entries
cd /tmp
cat > test_logging.cpp << 'EOF'
#include "logging.hpp"
int main() {
    ros2_dashboard::StructuredLogger::init("/tmp/test.log");
    auto& log = ros2_dashboard::StructuredLogger::instance();
    log.info("Test entry 1");
    log.warn("Test entry 2", {{"key", "value"}});
    return 0;
}
EOF

# View output
cat /tmp/test.log
# Should contain JSON lines with timestamps and metadata
```

## Configuration

### Environment Variables (Future Enhancement)

```bash
export ROS2_DASHBOARD_LOG_PATH=/var/log/dashboard.log
export ROS2_DASHBOARD_LOG_MAX_SIZE=104857600  # 100 MB
```

### Configuration File (Future Enhancement)

```json
{
    "logging": {
        "enabled": true,
        "path": "/var/log/ros2_dashboard/dashboard.log",
        "max_size_mb": 100,
        "retention_days": 30,
        "min_level": "info"
    }
}
```

## Log Analysis Tools

### Extract by Level

```bash
# Get all error entries
grep '"level":"error"' /var/log/dashboard.log | jq '.'
```

### Time Range Filtering

```bash
# Logs from last hour
start_ts=$(date -d '1 hour ago' +%s)000
grep "\"ts_ms\":[0-9]*" /var/log/dashboard.log | \
    jq --arg ts "$start_ts" 'select(.ts_ms > ($ts | tonumber))'
```

### Statistics

```bash
# Count by level
jq '.level' /var/log/dashboard.log | sort | uniq -c
```

## Compliance and Security

- **Log Privacy**: No passwords or sensitive keys logged by default
- **Audit Trail**: Immutable timestamp for compliance
- **Disk Space**: Automatic rotation prevents runaway file growth
- **Thread Safety**: No race conditions or data corruption

## Future Enhancements

- [ ] Async logging queue for high-throughput scenarios
- [ ] Compression of rotated log files (gzip)
- [ ] Network logging (syslog protocol)
- [ ] Log sampling (rate limiting)
- [ ] Structured log browser in GUI
- [ ] Export to ELK/Splunk
- [ ] Context propagation (request IDs, trace IDs)

## Files

- `include/logging.hpp` - Public API definition
- `src/logging.cpp` - Implementation with rotation logic
- `tests/test_logging.cpp` - Unit test suite (30+ assertions)

## Related Documentation

- [DEVELOPER.md](DEVELOPER.md) - Architecture and patterns
- [BUILD.md](BUILD.md) - Build configuration
