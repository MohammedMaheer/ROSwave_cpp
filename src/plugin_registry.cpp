/**
 * @file plugin_registry.cpp
 * @brief Implementation of plugin registry
 * @author Dashboard Team
 */

#include "plugin_registry.hpp"
#include "logging.hpp"

namespace ros2_dashboard::plugins {

PluginRegistry& PluginRegistry::instance() {
    static PluginRegistry registry;
    return registry;
}

void PluginRegistry::initialize(const std::string& plugin_dir) {
    loader_.add_search_path(plugin_dir);
    int loaded = loader_.load_all_plugins();
    StructuredLogger::instance().info("Plugin registry initialized with " + 
                                     std::to_string(loaded) + " plugins from " + plugin_dir);
}

void PluginRegistry::register_builtin_plugin(std::unique_ptr<IPlugin> plugin) {
    if (!plugin) {
        StructuredLogger::instance().warn("Attempted to register null plugin");
        return;
    }
    
    PluginMetadata metadata = plugin->get_metadata();
    builtin_plugins_[metadata.name] = std::move(plugin);
    StructuredLogger::instance().info("Registered built-in plugin: " + 
                                     metadata.display_name);
}

size_t PluginRegistry::total_plugins() const {
    return loader_.plugin_count() + builtin_plugins_.size();
}

std::vector<IPlugin*> PluginRegistry::get_by_type(PluginType type) const {
    auto result = loader_.get_plugins_by_type(type);
    
    // Add matching built-in plugins
    for (auto& p : builtin_plugins_) {
        if (p.second->get_metadata().type == type) {
            result.push_back(p.second.get());
        }
    }
    
    return result;
}

void PluginRegistry::print_registry_info() const {
    StructuredLogger::instance().info("=== Plugin Registry Info ===");
    StructuredLogger::instance().info("Total plugins: " + std::to_string(total_plugins()));
    
    // Loaded plugins
    auto all_loaded = loader_.get_all_plugins();
    if (!all_loaded.empty()) {
        StructuredLogger::instance().info("Loaded Plugins (" + std::to_string(all_loaded.size()) + "):");
        for (auto plugin : all_loaded) {
            auto meta = plugin->get_metadata();
            StructuredLogger::instance().info("  - " + meta.display_name + " v" + meta.version + 
                                             " (" + meta.author + ")");
        }
    }
    
    // Built-in plugins
    if (!builtin_plugins_.empty()) {
        StructuredLogger::instance().info("Built-in Plugins (" + 
                                         std::to_string(builtin_plugins_.size()) + "):");
        for (auto& p : builtin_plugins_) {
            auto meta = p.second->get_metadata();
            StructuredLogger::instance().info("  - " + meta.display_name + " v" + meta.version + 
                                             " (" + meta.author + ")");
        }
    }
}

}  // namespace ros2_dashboard::plugins
