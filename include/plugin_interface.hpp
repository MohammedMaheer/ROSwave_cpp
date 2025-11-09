/**
 * @file plugin_interface.hpp
 * @brief Plugin system interface for extending dashboard functionality
 * @author Dashboard Team
 */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <QWidget>
#include <QString>

namespace ros2_dashboard::plugins {

/**
 * @enum PluginType
 * @brief Classification of plugin functionality
 */
enum class PluginType {
    VISUALIZER,      // Custom visualization widget
    EXPORTER,        // Data export format
    ANALYZER,        // Data analysis tool
    FILTER,          // Topic/node filter
    TRANSFORMER,     // Data transformation
    LOGGER,          // Custom logging
    MONITOR,         // System monitoring
    CUSTOM           // Custom functionality
};

/**
 * @struct PluginMetadata
 * @brief Plugin information and capabilities
 */
struct PluginMetadata {
    std::string name;                ///< Unique plugin identifier
    std::string display_name;        ///< Human-readable name
    std::string version;             ///< Plugin version (semantic)
    std::string author;              ///< Plugin author
    std::string description;         ///< Brief description
    PluginType type;                 ///< Plugin functionality type
    std::vector<std::string> tags;   ///< Searchable tags
    int api_version = 1;             ///< Plugin API version compatibility
    bool enabled = true;             ///< Is plugin enabled by default?
};

/**
 * @struct PluginConfig
 * @brief Runtime configuration for plugin
 */
struct PluginConfig {
    std::string config_file;         ///< Path to config JSON (optional)
    std::string data_dir;            ///< Plugin data directory
    bool debug_mode = false;         ///< Enable debug output
    std::vector<std::string> args;   ///< Additional command-line arguments
};

/**
 * @class IPlugin
 * @brief Abstract interface for all plugins
 * 
 * Plugins must inherit from this class and implement all pure virtual methods.
 * Plugins are loaded at runtime and communicate with the dashboard via this interface.
 */
class IPlugin {
public:
    virtual ~IPlugin() = default;

    /**
     * @brief Get plugin metadata
     * @return Plugin information
     */
    virtual PluginMetadata get_metadata() const = 0;

    /**
     * @brief Initialize plugin
     * @param config Runtime configuration
     * @return true if initialization successful
     */
    virtual bool initialize(const PluginConfig& config) = 0;

    /**
     * @brief Cleanup plugin resources
     * @return true if cleanup successful
     */
    virtual bool shutdown() = 0;

    /**
     * @brief Check if plugin is ready to use
     */
    virtual bool is_ready() const = 0;

    /**
     * @brief Get plugin version
     */
    virtual std::string get_version() const { return "1.0.0"; }

    /**
     * @brief Execute plugin-specific action
     * @param action Action name
     * @param params Action parameters (format depends on action)
     * @return Result string
     */
    virtual std::string execute(const std::string& action, 
                               const std::string& params = "") { return ""; }
};

/**
 * @class IVisualizerPlugin
 * @brief Interface for visualization plugins
 */
class IVisualizerPlugin : public IPlugin {
public:
    /**
     * @brief Get the visualization widget
     * @return Qt widget to display (plugin manages lifetime)
     */
    virtual QWidget* get_widget() = 0;

    /**
     * @brief Update visualization with new data
     * @param data_json JSON formatted data
     */
    virtual void update_data(const std::string& data_json) = 0;

    /**
     * @brief Get current visualization state
     */
    virtual std::string get_state() const = 0;
};

/**
 * @class IExporterPlugin
 * @brief Interface for export plugins
 */
class IExporterPlugin : public IPlugin {
public:
    /**
     * @brief Get supported file extension (e.g., "csv", "json")
     */
    virtual std::string get_export_format() const = 0;

    /**
     * @brief Export data to specified file
     * @param filepath Output file path
     * @param data_json Data to export in JSON format
     * @return true if export successful
     */
    virtual bool export_data(const std::string& filepath, 
                            const std::string& data_json) = 0;
};

/**
 * @class IAnalyzerPlugin
 * @brief Interface for data analysis plugins
 */
class IAnalyzerPlugin : public IPlugin {
public:
    /**
     * @brief Analyze data
     * @param data_json Input data in JSON format
     * @return Analysis results in JSON format
     */
    virtual std::string analyze(const std::string& data_json) = 0;

    /**
     * @brief Get analysis report
     * @return Formatted report string
     */
    virtual std::string get_report() const = 0;
};

/**
 * @class IMonitorPlugin
 * @brief Interface for monitoring/watchdog plugins
 */
class IMonitorPlugin : public IPlugin {
public:
    /**
     * @brief Update monitor with metric data
     * @param metric_json System metrics in JSON
     */
    virtual void update_metrics(const std::string& metric_json) = 0;

    /**
     * @brief Check if any alerts should be raised
     * @return Alert message (empty if no alert)
     */
    virtual std::string check_alerts() = 0;
};

/**
 * @class IFilterPlugin
 * @brief Interface for filtering plugins
 */
class IFilterPlugin : public IPlugin {
public:
    /**
     * @brief Apply filter to data
     * @param data_json Input data
     * @return Filtered data in JSON format
     */
    virtual std::string apply_filter(const std::string& data_json) = 0;

    /**
     * @brief Set filter criteria
     * @param criteria_json Filter criteria in JSON
     */
    virtual void set_criteria(const std::string& criteria_json) = 0;
};

// Plugin factory function type
typedef IPlugin* (*PluginCreateFunc)();
typedef void (*PluginDestroyFunc)(IPlugin*);

}  // namespace ros2_dashboard::plugins
