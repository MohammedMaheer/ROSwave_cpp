/**
 * @file plugin_registry.hpp
 * @brief Global plugin registry and manager
 * @author Dashboard Team
 */

#pragma once

#include "plugin_interface.hpp"
#include "plugin_loader.hpp"
#include <memory>
#include <unordered_map>

namespace ros2_dashboard::plugins {

/**
 * @class PluginRegistry
 * @brief Singleton registry for all plugins
 * 
 * Provides centralized access to plugin loader and registry.
 * Thread-safe access to all plugin operations.
 */
class PluginRegistry {
public:
    /**
     * @brief Get singleton instance
     */
    static PluginRegistry& instance();

    /**
     * @brief Initialize the registry
     * @param plugin_dir Directory to search for plugins
     */
    void initialize(const std::string& plugin_dir);

    /**
     * @brief Get plugin loader
     */
    PluginLoader& loader() { return loader_; }

    /**
     * @brief Register a built-in plugin
     * @param plugin Pointer to plugin instance
     */
    void register_builtin_plugin(std::unique_ptr<IPlugin> plugin);

    /**
     * @brief Get count of all plugins (loaded + built-in)
     */
    size_t total_plugins() const;

    /**
     * @brief Get all plugins of type
     */
    std::vector<IPlugin*> get_by_type(PluginType type) const;

    /**
     * @brief Print registry info to logging
     */
    void print_registry_info() const;

private:
    PluginRegistry() = default;
    ~PluginRegistry() = default;

    PluginRegistry(const PluginRegistry&) = delete;
    PluginRegistry& operator=(const PluginRegistry&) = delete;

    PluginLoader loader_;
    std::unordered_map<std::string, std::unique_ptr<IPlugin>> builtin_plugins_;
};

}  // namespace ros2_dashboard::plugins
