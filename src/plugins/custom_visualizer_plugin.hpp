/**
 * @file custom_visualizer_plugin.hpp
 * @brief Example visualization plugin
 * @author Dashboard Team
 */

#pragma once

#include "plugin_interface.hpp"
#include <QWidget>
#include <QLabel>

/**
 * @class CustomVisualizerPlugin
 * @brief Example custom visualizer plugin implementation
 * 
 * Demonstrates the plugin interface for creating custom visualizations.
 * Shows how to:
 * - Inherit from IVisualizerPlugin
 * - Provide metadata
 * - Manage Qt widget lifecycle
 * - Update based on JSON data
 */
class CustomVisualizerPlugin : public ros2_dashboard::plugins::IVisualizerPlugin {
public:
    CustomVisualizerPlugin();
    ~CustomVisualizerPlugin() override;

    // IPlugin interface
    ros2_dashboard::plugins::PluginMetadata get_metadata() const override;
    bool initialize(const ros2_dashboard::plugins::PluginConfig& config) override;
    bool shutdown() override;
    bool is_ready() const override;
    std::string get_version() const override;
    std::string execute(const std::string& action, 
                       const std::string& params = "") override;

    // IVisualizerPlugin interface
    QWidget* get_widget() override;
    void update_data(const std::string& data_json) override;
    std::string get_state() const override;

private:
    bool ready_ = false;
    QWidget* widget_ = nullptr;
    QLabel* status_label_ = nullptr;
    std::string last_data_;
};
