/**
 * @file plugin_loader.cpp
 * @brief Implementation of dynamic plugin loading system
 * @author Dashboard Team
 */

#include "plugin_loader.hpp"
#include "logging.hpp"
#include <filesystem>
#include <algorithm>

namespace fs = std::filesystem;

namespace ros2_dashboard::plugins {

PluginLoader::PluginLoader() = default;

PluginLoader::~PluginLoader() {
    shutdown_all();
}

void PluginLoader::add_search_path(const std::string& directory) {
    if (fs::exists(directory) && fs::is_directory(directory)) {
        search_paths_.push_back(directory);
        StructuredLogger::instance().info("Added plugin search path: " + directory);
    } else {
        StructuredLogger::instance().warn("Plugin search path does not exist: " + directory);
    }
}

std::vector<std::string> PluginLoader::find_plugin_files_(const std::string& directory) {
    std::vector<std::string> plugins;
    
    try {
        for (const auto& entry : fs::directory_iterator(directory)) {
            const auto& path = entry.path();
            std::string filename = path.filename().string();
            
            // Check for plugin naming convention: lib*.so or *.dll
            bool is_plugin = false;
#ifdef _WIN32
            is_plugin = (path.extension() == ".dll");
#else
            is_plugin = (filename.find("lib") == 0 && 
                        path.extension() == ".so");
#endif
            
            if (is_plugin && entry.is_regular_file()) {
                plugins.push_back(path.string());
                StructuredLogger::instance().debug("Found plugin file: " + filename);
            }
        }
    } catch (const std::exception& e) {
        StructuredLogger::instance().warn("Error scanning plugin directory " + directory + 
                                         ": " + e.what());
    }
    
    return plugins;
}

bool PluginLoader::load_plugin_internal_(const std::string& file_path) {
    // Check if file exists
    if (!fs::exists(file_path)) {
        last_error_ = "Plugin file not found: " + file_path;
        StructuredLogger::instance().error(last_error_);
        return false;
    }

    // Try to open the plugin
    dlerror();  // Clear any previous errors
    void* handle = dlopen(file_path.c_str(), RTLD_LAZY);
    
    if (!handle) {
        last_error_ = std::string("Failed to load plugin: ") + dlerror();
        StructuredLogger::instance().error(last_error_);
        return false;
    }

    // Look for create function
    PluginCreateFunc create_func = 
        reinterpret_cast<PluginCreateFunc>(dlsym(handle, "create_plugin"));
    
    if (!create_func) {
        last_error_ = std::string("Plugin missing create_plugin function: ") + 
                     dlerror();
        StructuredLogger::instance().error(last_error_);
        dlclose(handle);
        return false;
    }

    // Create plugin instance
    IPlugin* plugin = create_func();
    if (!plugin) {
        last_error_ = "Failed to instantiate plugin";
        StructuredLogger::instance().error(last_error_);
        dlclose(handle);
        return false;
    }

    // Get metadata
    PluginMetadata metadata = plugin->get_metadata();
    
    // Check if plugin with same name already loaded
    if (plugins_.count(metadata.name)) {
        last_error_ = "Plugin with name already loaded: " + metadata.name;
        StructuredLogger::instance().warn(last_error_);
        
        // Clean up
        PluginDestroyFunc destroy_func = 
            reinterpret_cast<PluginDestroyFunc>(dlsym(handle, "destroy_plugin"));
        if (destroy_func) {
            destroy_func(plugin);
        }
        dlclose(handle);
        return false;
    }

    // Store plugin info
    LoadedPluginInfo info;
    info.plugin = std::unique_ptr<IPlugin>(plugin);
    info.handle = handle;
    info.metadata = metadata;
    info.file_path = file_path;
    
    plugins_[metadata.name] = std::move(info);
    
    StructuredLogger::instance().info("Successfully loaded plugin: " + metadata.display_name + 
                                     " v" + metadata.version);
    
    // Notify callbacks
    for (auto& callback : plugin_callbacks_) {
        callback(metadata, true);
    }
    
    return true;
}

bool PluginLoader::load_plugin(const std::string& file_path) {
    return load_plugin_internal_(file_path);
}

int PluginLoader::load_all_plugins() {
    int loaded_count = 0;
    
    for (const auto& search_path : search_paths_) {
        auto plugin_files = find_plugin_files_(search_path);
        
        for (const auto& file : plugin_files) {
            if (load_plugin_internal_(file)) {
                loaded_count++;
            }
        }
    }
    
    StructuredLogger::instance().info("Loaded " + std::to_string(loaded_count) + 
                                     " plugins total");
    return loaded_count;
}

bool PluginLoader::unload_plugin(const std::string& plugin_name) {
    auto it = plugins_.find(plugin_name);
    if (it == plugins_.end()) {
        last_error_ = "Plugin not found: " + plugin_name;
        return false;
    }

    auto& info = it->second;
    PluginMetadata metadata = info.metadata;
    
    // Shutdown if initialized
    if (info.initialized) {
        try {
            info.plugin->shutdown();
        } catch (const std::exception& e) {
            StructuredLogger::instance().error("Error shutting down plugin " + 
                                              plugin_name + ": " + e.what());
        }
    }

    // Call destroy function
    PluginDestroyFunc destroy_func = 
        reinterpret_cast<PluginDestroyFunc>(dlsym(info.handle, "destroy_plugin"));
    
    if (destroy_func) {
        destroy_func(info.plugin.get());
    }

    // Close library
    dlclose(info.handle);
    plugins_.erase(it);
    
    StructuredLogger::instance().info("Unloaded plugin: " + plugin_name);
    
    // Notify callbacks
    for (auto& callback : plugin_callbacks_) {
        callback(metadata, false);
    }
    
    return true;
}

IPlugin* PluginLoader::get_plugin(const std::string& plugin_name) {
    auto it = plugins_.find(plugin_name);
    if (it != plugins_.end()) {
        return it->second.plugin.get();
    }
    return nullptr;
}

std::vector<IPlugin*> PluginLoader::get_all_plugins() const {
    std::vector<IPlugin*> result;
    for (auto& p : plugins_) {
        result.push_back(p.second.plugin.get());
    }
    return result;
}

std::vector<IPlugin*> PluginLoader::get_plugins_by_type(PluginType type) const {
    std::vector<IPlugin*> result;
    for (auto& p : plugins_) {
        if (p.second.metadata.type == type) {
            result.push_back(p.second.plugin.get());
        }
    }
    return result;
}

int PluginLoader::initialize_all(const PluginConfig& config) {
    int initialized_count = 0;
    
    for (auto& p : plugins_) {
        auto& info = p.second;
        
        if (!info.initialized) {
            try {
                if (info.plugin->initialize(config)) {
                    info.initialized = true;
                    initialized_count++;
                    StructuredLogger::instance().info("Initialized plugin: " + 
                                                     info.metadata.name);
                } else {
                    StructuredLogger::instance().warn("Plugin initialization returned false: " + 
                                                     info.metadata.name);
                }
            } catch (const std::exception& e) {
                StructuredLogger::instance().error("Exception initializing plugin " + 
                                                  info.metadata.name + ": " + e.what());
            }
        }
    }
    
    StructuredLogger::instance().info("Initialized " + std::to_string(initialized_count) + 
                                     " plugins");
    return initialized_count;
}

void PluginLoader::shutdown_all() {
    std::vector<std::string> plugin_names;
    for (auto& p : plugins_) {
        plugin_names.push_back(p.first);
    }
    
    // Unload in reverse order (newest first)
    std::reverse(plugin_names.begin(), plugin_names.end());
    
    for (const auto& name : plugin_names) {
        unload_plugin(name);
    }
}

bool PluginLoader::has_plugin(const std::string& plugin_name) const {
    return plugins_.find(plugin_name) != plugins_.end();
}

}  // namespace ros2_dashboard::plugins
