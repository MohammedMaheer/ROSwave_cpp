# ROS2 Dashboard - Developer Guide

## Table of Contents
1. [Architecture Overview](#architecture-overview)
2. [Module Design](#module-design)
3. [Code Structure](#code-structure)
4. [Extension Points](#extension-points)
5. [Testing Strategy](#testing-strategy)
6. [Code Standards](#code-standards)
7. [Debugging](#debugging)

## Architecture Overview

### Design Principles
- **Separation of Concerns**: Core logic isolated from UI
- **Thread Safety**: All shared data protected with mutexes
- **Caching Strategy**: LRU + TTL for efficient discovery
- **Offline-First**: Local persistence for reliability
- **Non-Blocking UI**: Async worker threads for long operations

### Component Interaction

```
┌──────────────────┐
│   GUI Layer      │ (Qt Widgets - Main Thread)
│ (Tab Views)      │
└────────┬─────────┘
         │ Qt Signals/Slots
         ▼
┌──────────────────────────────────────────┐
│      Main Window (Coordinator)           │
│  - Manages tab lifecycle                 │
│  - Handles settings                      │
│  - Coordinates updates                   │
└────────┬─────────────────────────────────┘
         │ Ownership
         ▼
┌──────────────────────────────────────────┐
│     Core Managers (Thread-Safe)          │
│                                          │
│ ┌──────────────────────────────────────┐ │
│ │ ROS2Manager                          │ │
│ │ - Cache (LRU + TTL)                  │ │
│ │ - ROS2 Discovery                     │ │
│ │ - Bag Recording Control              │ │
│ └──────────────────────────────────────┘ │
│                                          │
│ ┌──────────────────────────────────────┐ │
│ │ MetricsCollector                     │ │
│ │ - Metrics Aggregation                │ │
│ │ - Historical Data                    │ │
│ │ - Export (CSV/JSON)                  │ │
│ └──────────────────────────────────────┘ │
│                                          │
│ ┌──────────────────────────────────────┐ │
│ │ NetworkManager                       │ │
│ │ - SQLite Queue Persistence           │ │
│ │ - Upload Management                  │ │
│ │ - Retry Logic                        │ │
│ └──────────────────────────────────────┘ │
│                                          │
│ ┌──────────────────────────────────────┐ │
│ │ MLExporter                           │ │
│ │ - Dataset Packaging                  │ │
│ │ - Metadata Generation                │ │
│ │ - Archive Creation                   │ │
│ └──────────────────────────────────────┘ │
└──────────────────────────────────────────┘
         │
         ▼
┌──────────────────────────────────────────┐
│      AsyncWorker (Thread Pool)           │
│ - 1-4 worker threads                    │
│ - Task queue with futures               │
│ - Non-blocking execution                │
└──────────────────────────────────────────┘
```

## Module Design

### ROS2Manager

**Purpose:** Discover and manage ROS2 system

**Key Responsibilities:**
- Discover topics, nodes, services using `ros2` CLI
- Cache results with TTL-based invalidation
- Start/stop rosbag2 recording
- Extract bag metadata
- Track cache hit ratio

**Thread Safety:**
- All public methods thread-safe
- Internal cache protected by mutex
- Fast-fail timeout (1s) on CLI calls

**Cache Strategy:**
```cpp
// LRU + TTL Implementation
struct CacheEntry<T> {
    T data;
    std::chrono::steady_clock::time_point timestamp;
    bool is_valid(int ttl_seconds) {
        auto age = now - timestamp;
        return age < ttl_seconds;
    }
};
```

**Performance Targets:**
- Topic discovery: < 500ms
- Cache hit ratio: > 75%

**Usage Example:**
```cpp
auto ros2_mgr = std::make_unique<ROS2Manager>();
ros2_mgr->set_cache_ttl(5);  // 5 second TTL

auto topics = ros2_mgr->get_topics();  // Cached
auto ratio = ros2_mgr->get_cache_hit_ratio();
```

### MetricsCollector

**Purpose:** Collect and maintain real-time metrics

**Key Responsibilities:**
- Collect system metrics (/proc parsing)
- Collect recording metrics (IPC)
- Track metrics history (default 1 hour)
- Export to CSV/JSON
- Thread-safe metric updates

**Metrics Collected:**
- CPU usage, memory, disk, thermal
- Write speed, message rate, compression ratio
- UI frame rate, cache hit ratio

**History Management:**
- Stores last N samples (default 3600 = 1 hour at 1s intervals)
- Auto-trims when exceeds max
- Supports historical queries by time period

**Usage Example:**
```cpp
auto metrics = std::make_unique<MetricsCollector>();
auto snapshot = metrics->get_snapshot();
metrics->export_csv("metrics.csv", 3600);  // Export 1 hour

// Update metrics (called by recording/network components)
metrics->update_recording_metrics(25.5, 1000, 0.6, 1073741824);
```

### NetworkManager

**Purpose:** Manage offline-first cloud uploads

**Key Responsibilities:**
- Persist upload queue to SQLite
- Handle chunked uploads with resume
- Implement exponential backoff retry
- Manage upload priority
- Bandwidth throttling
- Update metrics during uploads

**Queue Persistence:**
- SQLite database stores task state
- Queue survives application restarts
- Automatic cleanup of old completed tasks

**Upload Flow:**
1. User adds file to queue
2. Task stored in SQLite with status=PENDING
3. Background worker processes queue
4. Chunks uploaded with progress callbacks
5. On success: mark COMPLETED, emit callback
6. On failure: increment retry count, backoff, retry

**Retry Strategy:**
```cpp
max_retries = 3
backoff = 2^(attempt) seconds
attempt 0: immediate retry
attempt 1: wait 2s
attempt 2: wait 4s
```

**Usage Example:**
```cpp
auto net_mgr = std::make_unique<NetworkManager>();
net_mgr->initialize("uploads.db");
net_mgr->set_endpoint("http://server:8080", "api-key");

auto task_id = net_mgr->queue_upload("/path/to/bag", 5);
net_mgr->start_queue_processing(2);  // Max 2 concurrent

net_mgr->set_progress_callback([](const std::string& id, int64_t up, int64_t total) {
    std::cout << "Upload " << id << ": " << (up*100/total) << "%" << std::endl;
});
```

### MLExporter

**Purpose:** Package data for machine learning

**Key Responsibilities:**
- Extract bag metadata
- Generate metadata.json (recording info)
- Generate schema.json (message types)
- Create .tar.gz archives
- Batch operations with progress tracking

**Export Format:**
```
export_dir/
├── bag1.db3
├── bag2.db3
├── metadata.json
├── schema.json
└── export_dir.tar.gz
```

**Metadata Example:**
```json
{
  "export_version": "1.0",
  "export_date": "2025-01-15T10:30:00Z",
  "bag_files": ["recording.db3"],
  "topics": ["/sensor/lidar"],
  "message_counts": {"/sensor/lidar": 15000},
  "total_duration": "00:10:23",
  "total_size_bytes": 5368709120
}
```

### AsyncWorker

**Purpose:** Thread pool for non-blocking operations

**Key Responsibilities:**
- Manage worker thread lifecycle
- Queue tasks with futures
- Support callbacks
- Track pending task count

**Thread Pool:**
- Configurable size (1-4, default 2)
- One condition variable for signaling
- Thread-safe task queue

**Usage Examples:**
```cpp
auto worker = std::make_unique<AsyncWorker>(2);

// Async task with future
auto future = worker->enqueue([]() {
    return expensive_operation();
});

// Later...
auto result = future.get();  // Blocks until ready

// Task with callback
worker->enqueue_with_callback(
    []() { return compute(); },
    [](auto result) { process_result(result); }
);

// Query state
auto pending = worker->get_pending_tasks();
auto threads = worker->get_thread_count();
```

## Code Structure

### Header Organization

**Include Guards:**
```cpp
#pragma once  // Preferred over #ifndef
#include <string>
#include <vector>
#include <mutex>
#include <memory>
```

**Forward Declarations:**
- Use for reducing compile dependencies
- Example: `class ROS2Manager;` in header

**Namespace Organization:**
```cpp
namespace ros2_dashboard {
    // Core classes
    class ROS2Manager { ... };
    class MetricsCollector { ... };
}

namespace ros2_dashboard::gui {
    // GUI classes
    class MainWindow : public QMainWindow { ... };
}
```

### Const Correctness

```cpp
// Getters should be const
double get_cache_hit_ratio() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return hit_ratio_;
}

// Methods that don't modify should be const
std::vector<TopicInfo> get_topics() const;  // WRONG! Modifies cache
std::vector<TopicInfo> get_topics();         // CORRECT
```

### Memory Management

**Prefer Unique Pointers:**
```cpp
// Good - clear ownership
std::unique_ptr<ROS2Manager> manager_;

// Shared ownership (rare)
std::shared_ptr<AsyncWorker> worker_;

// Avoid raw pointers in new code
ROS2Manager* manager;  // AVOID
```

## Extension Points

### Adding a New Tab

1. **Create Header** (`include/gui/new_tab.hpp`):
```cpp
class NewTab : public QWidget {
    Q_OBJECT
public:
    void initialize(std::shared_ptr<CoreManager> manager,
                    std::shared_ptr<AsyncWorker> worker);
    void refresh();
private slots:
    void on_action_triggered();
private:
    void create_ui_();
    std::shared_ptr<CoreManager> manager_;
};
```

2. **Create Implementation** (`src/gui/new_tab.cpp`):
- Implement UI creation
- Connect signals/slots
- Add async refresh logic

3. **Register in MainWindow**:
```cpp
new_tab_ = std::make_unique<NewTab>();
new_tab_->initialize(core_manager, async_worker);
tab_widget_->addTab(new_tab_.get(), "New Tab");
```

### Adding a New Metric

1. **Extend MetricsSnapshot**:
```cpp
struct MetricsSnapshot {
    // ... existing fields
    CustomMetric my_metric;
};
```

2. **Add Update Method**:
```cpp
void update_custom_metric(const CustomMetric& value) {
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    current_snapshot_.my_metric = value;
}
```

3. **Add Collection Logic**:
```cpp
MetricsSnapshot get_snapshot() {
    // ... existing code
    snapshot.my_metric = collect_custom_metric_();
    return snapshot;
}
```

### Implementing Custom Export Format

1. **Extend MLExporter**:
```cpp
bool export_custom_format(const std::string& bag_path,
                         const std::string& output_path) {
    // Custom export logic
    return true;
}
```

2. **Call from GUI Tab**:
```cpp
ml_exporter_->export_custom_format(bag, output);
```

## Testing Strategy

### Unit Tests

**Test Structure:**
```cpp
#include <gtest/gtest.h>
#include "ros2_manager.hpp"

class ROS2ManagerTest : public ::testing::Test {
protected:
    ROS2Manager manager_;
};

TEST_F(ROS2ManagerTest, CacheInvalidation) {
    manager_.set_cache_ttl(1);
    auto topics1 = manager_.get_topics();
    auto topics2 = manager_.get_topics();  // Should be cached
    EXPECT_GT(manager_.get_cache_hit_ratio(), 0.5);
}
```

**Key Test Areas:**
- Cache behavior (TTL, LRU, hit ratio)
- Thread safety (concurrent access)
- Error handling (invalid inputs)
- Performance (timing constraints)
- Data integrity (export formats)

### Integration Tests

**End-to-End Workflow:**
1. Start application
2. Discover ROS2 system
3. Record bag file
4. Export dataset
5. Upload to server
6. Verify on server

### Performance Verification

Run benchmark tool:
```bash
./ros2_verify_performance --export --output results.json
```

Verify results meet targets (see PERFORMANCE.md).

## Code Standards

### Google C++ Style Guide

**File Organization:**
- License comment
- Include guards
- Includes (system, then project)
- Forward declarations
- Class/struct definitions
- Inline implementations

**Naming:**
- Classes: `PascalCase`
- Functions/methods: `snake_case`
- Member variables: `trailing_underscore_`
- Constants: `kConstantName`

**Comments:**
```cpp
// Use // for single-line comments
// Function comments explain what, not how

/**
 * @brief Brief description
 * @param param Description
 * @return Description
 */
void important_function(int param);
```

**Error Handling:**
```cpp
// Use exceptions for exceptional situations
if (!initialize()) {
    throw std::runtime_error("Failed to initialize");
}

// Use return values for expected errors
bool success = start_recording();
if (!success) {
    log_error("Recording failed");
}
```

### Line Length
- Maximum 80 characters for code
- Maximum 100 characters for comments/documentation
- Break long lines at logical points

### Formatting
```cpp
// Braces on same line
if (condition) {
    statement;
}

// Spaces around operators
int result = a + b * c;

// No spaces before function parentheses
function(arg1, arg2);
```

## Debugging

### Enable Debug Build
```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)
```

### GDB Debugging
```bash
gdb ./ros2_dashboard
(gdb) run
(gdb) bt  # Backtrace
(gdb) print variable_name
(gdb) watch expression
```

### Valgrind Memory Check
```bash
valgrind --leak-check=full \
    --show-leak-kinds=all \
    ./ros2_dashboard
```

### Enable Debug Logging

Edit `config.json`:
```json
{
  "ui": {
    "debug_logging": true
  }
}
```

Or enable at runtime:
```cpp
ENABLE_DEBUG_LOGGING();
```

### Common Issues

**Segmentation Fault:**
```bash
# Run under GDB with catch fork
gdb --args ./ros2_dashboard
(gdb) catch fork
(gdb) run
```

**Data Race (Thread Synchronization):**
```bash
# Use ThreadSanitizer
cmake -DCMAKE_CXX_FLAGS="-fsanitize=thread" ..
```

**Memory Leak:**
```bash
# Run with Valgrind
valgrind --leak-check=full ./ros2_dashboard
```

## Building Documentation

### Generate Doxygen
```bash
doxygen Doxyfile
# Output in docs/api/
```

### Build User Manual PDF
```bash
# Requires pandoc
pandoc docs/USER_MANUAL.md -o docs/USER_MANUAL.pdf
```

## Performance Profiling

### CPU Profiling
```bash
# Using perf
perf record ./ros2_dashboard
perf report

# Using gprof
g++ -pg -o executable source.cpp
./executable
gprof executable gmon.out
```

### Memory Profiling
```bash
# Using Massif
valgrind --tool=massif ./ros2_dashboard
ms_print massif.out.<pid>
```

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for:
- Pull request process
- Code review guidelines
- Commit message format
- Testing requirements

## Related Documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) - Detailed architecture
- [PERFORMANCE.md](PERFORMANCE.md) - Performance targets
- [TESTING.md](TESTING.md) - Testing procedures
