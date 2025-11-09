# Plugin Architecture Guide

## Overview

The ROS2 Dashboard plugin system enables extensibility through dynamic plugin loading. Plugins can add new visualizations, export formats, analyzers, and other functionality without modifying the core codebase.

## Plugin Types

### 1. **VisualizerPlugin** (`IVisualizerPlugin`)
Create custom visualization widgets for data display.

**Key Methods:**
- `QWidget* get_widget()` - Return visualization widget
- `void update_data(const std::string& data_json)` - Update with new data
- `std::string get_state() const` - Get current state

**Use Cases:**
- Custom chart types
- Domain-specific visualizations
- Live data streaming displays

### 2. **ExporterPlugin** (`IExporterPlugin`)
Export dashboard data to various formats.

**Key Methods:**
- `std::string get_export_format() const` - Format identifier (e.g., "csv")
- `bool export_data(const std::string& filepath, const std::string& data_json)` - Export data

**Use Cases:**
- CSV/Excel export
- PDF report generation
- Custom binary formats

### 3. **AnalyzerPlugin** (`IAnalyzerPlugin`)
Perform analysis on collected data.

**Key Methods:**
- `std::string analyze(const std::string& data_json)` - Analyze data
- `std::string get_report() const` - Get analysis results

**Use Cases:**
- Statistical analysis
- Anomaly detection
- Pattern recognition

### 4. **MonitorPlugin** (`IMonitorPlugin`)
Monitor system metrics and raise alerts.

**Key Methods:**
- `void update_metrics(const std::string& metric_json)` - Update metrics
- `std::string check_alerts() const` - Check for alert conditions

**Use Cases:**
- Custom monitoring rules
- Integration with external monitoring systems
- Performance profiling

### 5. **FilterPlugin** (`IFilterPlugin`)
Filter or transform topic/node data.

**Key Methods:**
- `std::string apply_filter(const std::string& data_json)` - Apply filter
- `void set_criteria(const std::string& criteria_json)` - Configure filter

**Use Cases:**
- Topic filtering
- Data transformation
- Message validation

## Creating a Plugin

### Step 1: Create Header File

```cpp
#pragma once
#include "plugin_interface.hpp"
#include <QWidget>

class MyCustomPlugin : public ros2_dashboard::plugins::IVisualizerPlugin {
public:
    ros2_dashboard::plugins::PluginMetadata get_metadata() const override;
    bool initialize(const ros2_dashboard::plugins::PluginConfig& config) override;
    bool shutdown() override;
    bool is_ready() const override;
    
    QWidget* get_widget() override;
    void update_data(const std::string& data_json) override;
    std::string get_state() const override;

private:
    bool ready_ = false;
    QWidget* widget_ = nullptr;
};
```

### Step 2: Implement Plugin Class

```cpp
#include "my_custom_plugin.hpp"
#include <QVBoxLayout>
#include <QLabel>

ros2_dashboard::plugins::PluginMetadata MyCustomPlugin::get_metadata() const {
    return {
        .name = "my_custom_plugin",
        .display_name = "My Custom Plugin",
        .version = "1.0.0",
        .author = "Your Name",
        .description = "Description of what this plugin does",
        .type = ros2_dashboard::plugins::PluginType::VISUALIZER,
        .tags = {"custom", "visualization"},
        .api_version = 1,
        .enabled = true
    };
}

bool MyCustomPlugin::initialize(const ros2_dashboard::plugins::PluginConfig& config) {
    widget_ = new QWidget();
    auto layout = new QVBoxLayout(widget_);
    
    auto label = new QLabel("My Custom Plugin");
    layout->addWidget(label);
    
    ready_ = true;
    return true;
}

bool MyCustomPlugin::shutdown() {
    ready_ = false;
    return true;
}

bool MyCustomPlugin::is_ready() const {
    return ready_;
}

QWidget* MyCustomPlugin::get_widget() {
    return widget_;
}

void MyCustomPlugin::update_data(const std::string& data_json) {
    // Handle data updates
}

std::string MyCustomPlugin::get_state() const {
    return "{}";
}
```

### Step 3: Export Factory Functions

**CRITICAL**: Every plugin must export these C functions for dynamic loading:

```cpp
extern "C" {
    ros2_dashboard::plugins::IPlugin* create_plugin() {
        return new MyCustomPlugin();
    }

    void destroy_plugin(ros2_dashboard::plugins::IPlugin* plugin) {
        delete plugin;
    }
}
```

### Step 4: Create CMakeLists.txt

```cmake
# Build plugin as shared library
add_library(my_custom_plugin SHARED my_custom_plugin.cpp)

target_link_libraries(my_custom_plugin
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
)

target_compile_options(my_custom_plugin PRIVATE -fPIC -Wall -Wextra)

# Install to plugins directory
install(TARGETS my_custom_plugin
    LIBRARY DESTINATION lib/ros2_dashboard/plugins
)
```

### Step 5: Build Plugin

```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

## Plugin Lifecycle

### 1. **Discovery Phase**
Plugin system scans configured directories for `.so` files matching pattern `lib*.so`.

### 2. **Loading Phase**
- `dlopen()` loads the shared library
- Plugin factory function `create_plugin()` is called
- Plugin instance created
- Metadata retrieved

### 3. **Initialization Phase**
- `initialize()` called with runtime configuration
- Plugin can create UI, spawn threads, connect signals
- Plugin should return `true` if successful

### 4. **Active Phase**
- Plugin receives data updates via interface methods
- Plugin can emit signals/callbacks
- Dashboard displays plugin widget

### 5. **Shutdown Phase**
- `shutdown()` called
- Plugin should clean up resources
- `destroy_plugin()` called to deallocate memory
- `dlclose()` unloads shared library

## Plugin Configuration

Plugins receive `PluginConfig` during initialization:

```cpp
struct PluginConfig {
    std::string config_file;      // Path to JSON config
    std::string data_dir;         // Plugin data directory
    bool debug_mode;              // Enable debug logging
    std::vector<std::string> args; // Command-line arguments
};
```

Load config from JSON:

```cpp
bool MyPlugin::initialize(const ros2_dashboard::plugins::PluginConfig& config) {
    nlohmann::json cfg;
    std::ifstream file(config.config_file);
    if (file.is_open()) {
        file >> cfg;
        // Use config values
    }
    
    // Create data directory if needed
    std::filesystem::create_directories(config.data_dir);
    
    return true;
}
```

## Data Format (JSON)

### Metrics Data
```json
{
    "timestamp": 1673098765000,
    "cpu_percent": 45.2,
    "memory_mb": 1024,
    "temperature_c": 65.5
}
```

### Topic Data
```json
{
    "topics": [
        {
            "name": "/sensor/imu",
            "type": "sensor_msgs/msg/Imu",
            "publisher_count": 1,
            "subscriber_count": 2,
            "frequency_hz": 100.0
        }
    ]
}
```

## Best Practices

### 1. **Thread Safety**
- Use Qt signals/slots for thread-safe UI updates
- Protect shared data with mutexes
- Avoid blocking UI thread

```cpp
void MyPlugin::update_data(const std::string& data_json) {
    // Do NOT update UI directly from worker thread
    QMetaObject::invokeMethod(this, [this, data_json]() {
        // Update UI in main thread
        update_ui_with(data_json);
    });
}
```

### 2. **Error Handling**
- Return false from initialize/shutdown on error
- Use structured logging
- Don't throw exceptions into dashboard

```cpp
bool MyPlugin::initialize(const PluginConfig& config) {
    try {
        // ... initialization code ...
    } catch (const std::exception& e) {
        std::cerr << "Plugin init failed: " << e.what() << std::endl;
        return false;
    }
    return true;
}
```

### 3. **Memory Management**
- Use `std::unique_ptr` for owned objects
- Properly clean up in shutdown()
- Avoid memory leaks in signal handlers

### 4. **Performance**
- Cache expensive computations
- Use adaptive algorithms
- Monitor memory usage
- Profile with tools like valgrind

### 5. **Documentation**
- Document JSON data format expected
- Provide example configurations
- Document required dependencies
- Explain any external APIs used

## API Reference

### PluginMetadata
```cpp
struct PluginMetadata {
    std::string name;                // Unique identifier
    std::string display_name;        // Human-readable name
    std::string version;             // Semantic version
    std::string author;              // Plugin author
    std::string description;         // Brief description
    PluginType type;                 // VISUALIZER, EXPORTER, etc.
    std::vector<std::string> tags;   // Search tags
    int api_version = 1;             // API compatibility
    bool enabled = true;             // Enabled by default
};
```

### IPlugin (Base Interface)
```cpp
virtual PluginMetadata get_metadata() const = 0;
virtual bool initialize(const PluginConfig& config) = 0;
virtual bool shutdown() = 0;
virtual bool is_ready() const = 0;
virtual std::string get_version() const;
virtual std::string execute(const std::string& action, const std::string& params = "");
```

## Example: Custom CSV Exporter

```cpp
#include "plugin_interface.hpp"
#include <fstream>

class CSVExporterPlugin : public ros2_dashboard::plugins::IExporterPlugin {
public:
    PluginMetadata get_metadata() const override {
        return {
            .name = "csv_exporter",
            .display_name = "CSV Exporter",
            .version = "1.0.0",
            .author = "Dashboard Team",
            .description = "Export metrics to CSV format",
            .type = ros2_dashboard::plugins::PluginType::EXPORTER,
            .tags = {"export", "csv"}
        };
    }

    bool initialize(const PluginConfig& config) override { return true; }
    bool shutdown() override { return true; }
    bool is_ready() const override { return true; }

    std::string get_export_format() const override { return "csv"; }

    bool export_data(const std::string& filepath, 
                    const std::string& data_json) override {
        std::ofstream file(filepath);
        if (!file.is_open()) return false;

        nlohmann::json data = nlohmann::json::parse(data_json);
        
        // Write CSV header
        file << "timestamp,cpu,memory,temperature\n";
        
        // Write data rows
        for (const auto& point : data["metrics"]) {
            file << point["timestamp"] << ","
                 << point["cpu"] << ","
                 << point["memory"] << ","
                 << point["temperature"] << "\n";
        }

        file.close();
        return true;
    }
};

extern "C" {
    ros2_dashboard::plugins::IPlugin* create_plugin() {
        return new CSVExporterPlugin();
    }

    void destroy_plugin(ros2_dashboard::plugins::IPlugin* plugin) {
        delete plugin;
    }
}
```

## Troubleshooting

### Plugin Not Loaded
- Check plugin file naming (must be `lib*.so`)
- Verify search paths configured
- Check dlopen error: `PluginLoader::get_last_error()`
- Ensure factory functions exported with `extern "C"`

### Symbol Undefined
- Verify all dependencies linked (`target_link_libraries`)
- Use `nm` to check exported symbols: `nm -D libplugin.so`
- Check for version mismatches

### Segmentation Fault
- Enable ASAN: `cmake -DCMAKE_BUILD_TYPE=ASAN ..`
- Use GDB: `gdb ros2_dashboard`
- Check null pointer dereferences

### Memory Leaks
- Use valgrind: `valgrind --leak-check=full ros2_dashboard`
- Ensure proper cleanup in shutdown()
- Use smart pointers instead of raw pointers

## Registry API

### Register Built-in Plugin
```cpp
auto plugin = std::make_unique<MyPlugin>();
PluginRegistry::instance().register_builtin_plugin(std::move(plugin));
```

### Load Dynamic Plugin
```cpp
PluginRegistry::instance().loader().load_plugin("/path/to/libplugin.so");
```

### Get Plugins by Type
```cpp
auto visualizers = PluginRegistry::instance().get_by_type(
    PluginType::VISUALIZER
);
```

## Advanced Topics

### Plugin Versioning
- Use semantic versioning in metadata
- API version field ensures compatibility
- Maintain backward compatibility when possible

### Plugin Dependencies
- List dependencies in metadata `tags`
- Document required packages in README
- Consider optional dependency fallbacks

### Plugin Communication
- Use Qt signals for inter-plugin communication
- Registry provides central plugin access point
- JSON data format enables loose coupling

## See Also
- `include/plugin_interface.hpp` - Interface definitions
- `include/plugin_loader.hpp` - Loading mechanism
- `include/plugin_registry.hpp` - Registry API
- `src/plugins/custom_visualizer_plugin.*` - Example implementation
