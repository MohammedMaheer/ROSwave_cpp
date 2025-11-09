/**
 * @file plugin_loader.hpp
 * @brief Dynamic plugin loading and management system
 * @author Dashboard Team
 */

#pragma once

#include "plugin_interface.hpp"
#include <map>
#include <memory>
#include <vector>
#include <functional>
#include <dlfcn.h>

namespace ros2_dashboard::plugins {

/**
 * @struct LoadedPluginInfo
 * @brief Information about a loaded plugin
 */
struct LoadedPluginInfo {
    std::unique_ptr<IPlugin> plugin;
    void* handle;                   // dlopen handle
    PluginMetadata metadata;
    std::string file_path;
    bool initialized = false;
};

/**
 * @class PluginLoader
 * @brief Loads and manages dynamic plugins
 * 
 * Handles:
 * - Dynamic loading of .so/.dll files
 * - Plugin instantiation and lifecycle
 * - Plugin registry and discovery
 * - Error handling and logging
 */
class PluginLoader {
public:
    using PluginCallback = std::function<void(const PluginMetadata&, bool loaded)>;

    PluginLoader();
    ~PluginLoader();

    // Delete copy operations, allow move
    PluginLoader(const PluginLoader&) = delete;
    PluginLoader& operator=(const PluginLoader&) = delete;
    PluginLoader(PluginLoader&&) = default;
    PluginLoader& operator=(PluginLoader&&) = default;

    /**
     * @brief Add plugin search directory
     * @param directory Path to search for plugins
     */
    void add_search_path(const std::string& directory);

    /**
     * @brief Load plugin from file
     * @param file_path Path to plugin .so/.dll file
     * @return true if loaded successfully
     */
    bool load_plugin(const std::string& file_path);

    /**
     * @brief Load all plugins from search directories
     * @return Number of plugins loaded
     */
    int load_all_plugins();

    /**
     * @brief Unload specific plugin
     * @param plugin_name Plugin identifier from metadata
     */
    bool unload_plugin(const std::string& plugin_name);

    /**
     * @brief Get loaded plugin by name
     * @param plugin_name Plugin identifier
     * @return Pointer to plugin (null if not loaded)
     */
    IPlugin* get_plugin(const std::string& plugin_name);

    /**
     * @brief Get all loaded plugins
     */
    std::vector<IPlugin*> get_all_plugins() const;

    /**
     * @brief Get all plugins of specific type
     */
    std::vector<IPlugin*> get_plugins_by_type(PluginType type) const;

    /**
     * @brief Initialize all loaded plugins
     * @param config Configuration for plugins
     * @return Number of successfully initialized plugins
     */
    int initialize_all(const PluginConfig& config);

    /**
     * @brief Shutdown all plugins
     */
    void shutdown_all();

    /**
     * @brief Get loaded plugin count
     */
    size_t plugin_count() const { return plugins_.size(); }

    /**
     * @brief Check if plugin is loaded
     */
    bool has_plugin(const std::string& plugin_name) const;

    /**
     * @brief Register callback for plugin load/unload events
     */
    void on_plugin_loaded(PluginCallback callback) { 
        plugin_callbacks_.push_back(callback); 
    }

    /**
     * @brief Get last error message
     */
    std::string get_last_error() const { return last_error_; }

private:
    /**
     * @brief Load plugin from single file
     */
    bool load_plugin_internal_(const std::string& file_path);

    /**
     * @brief Find plugin library files in directory
     */
    std::vector<std::string> find_plugin_files_(const std::string& directory);

    std::map<std::string, LoadedPluginInfo> plugins_;
    std::vector<std::string> search_paths_;
    std::vector<PluginCallback> plugin_callbacks_;
    std::string last_error_;
};

}  // namespace ros2_dashboard::plugins
