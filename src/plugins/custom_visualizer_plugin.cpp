/**
 * @file custom_visualizer_plugin.cpp
 * @brief Implementation of example visualization plugin
 * @author Dashboard Team
 */

#include "custom_visualizer_plugin.hpp"
#include <QVBoxLayout>
#include <QTextEdit>
#include <QPushButton>

CustomVisualizerPlugin::CustomVisualizerPlugin() = default;

CustomVisualizerPlugin::~CustomVisualizerPlugin() {
    if (widget_) {
        delete widget_;
    }
}

ros2_dashboard::plugins::PluginMetadata CustomVisualizerPlugin::get_metadata() const {
    return {
        .name = "custom_visualizer",
        .display_name = "Custom Visualizer",
        .version = "1.0.0",
        .author = "Dashboard Team",
        .description = "Example custom visualization plugin demonstrating plugin API",
        .type = ros2_dashboard::plugins::PluginType::VISUALIZER,
        .tags = {"visualization", "example", "demo"},
        .api_version = 1,
        .enabled = true
    };
}

bool CustomVisualizerPlugin::initialize(
    const ros2_dashboard::plugins::PluginConfig& config) {
    
    // Create visualization widget
    widget_ = new QWidget();
    widget_->setWindowTitle("Custom Visualizer Plugin");
    
    auto layout = new QVBoxLayout(widget_);
    
    status_label_ = new QLabel("Plugin initialized and ready");
    status_label_->setStyleSheet("color: green; font-weight: bold;");
    layout->addWidget(status_label_);
    
    auto data_display = new QTextEdit();
    data_display->setReadOnly(true);
    data_display->setPlaceholderText("Data updates will appear here...");
    layout->addWidget(data_display);
    
    auto info_label = new QLabel(
        "This is an example plugin that displays custom visualization data.\n"
        "It receives JSON data and can be extended for specialized visualization."
    );
    info_label->setWordWrap(true);
    layout->addWidget(info_label);
    
    widget_->setLayout(layout);
    ready_ = true;
    
    return true;
}

bool CustomVisualizerPlugin::shutdown() {
    ready_ = false;
    return true;
}

bool CustomVisualizerPlugin::is_ready() const {
    return ready_;
}

std::string CustomVisualizerPlugin::get_version() const {
    return "1.0.0";
}

std::string CustomVisualizerPlugin::execute(const std::string& action, 
                                            const std::string& params) {
    if (action == "status") {
        return "{\"status\": \"ready\", \"data_points\": 0}";
    } else if (action == "reset") {
        last_data_.clear();
        return "{\"result\": \"reset_complete\"}";
    }
    return "{\"error\": \"unknown_action\"}";
}

QWidget* CustomVisualizerPlugin::get_widget() {
    return widget_;
}

void CustomVisualizerPlugin::update_data(const std::string& data_json) {
    if (!ready_ || !status_label_) return;
    
    last_data_ = data_json;
    status_label_->setText(QString::fromStdString(
        "Last update: " + data_json.substr(0, 50) + "..."
    ));
}

std::string CustomVisualizerPlugin::get_state() const {
    return last_data_;
}

// Plugin factory functions
extern "C" {
    ros2_dashboard::plugins::IPlugin* create_plugin() {
        return new CustomVisualizerPlugin();
    }

    void destroy_plugin(ros2_dashboard::plugins::IPlugin* plugin) {
        delete plugin;
    }
}
