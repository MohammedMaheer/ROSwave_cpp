/**
 * @file gui/settings_dialog.cpp
 */

#include "gui/settings_dialog.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabWidget>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QSettings>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFile>
#include <iostream>

namespace ros2_dashboard::gui {

SettingsDialog::SettingsDialog(QWidget* parent)
    : QDialog(parent) {
    setWindowTitle("Settings");
    create_ui_();
    load_settings_();
}

SettingsDialog::~SettingsDialog() = default;

void SettingsDialog::on_apply_clicked() {
    save_settings_();
    accept();
}

void SettingsDialog::on_reset_clicked() {
    load_settings_();
}

void SettingsDialog::create_ui_() {
    auto main_layout = new QVBoxLayout();
    auto tab_widget = new QTabWidget();

    // Cache settings tab
    auto cache_widget = new QWidget();
    auto cache_layout = new QVBoxLayout();
    auto cache_group = new QGroupBox("Cache Configuration");
    auto cache_inner = new QVBoxLayout();
    cache_ttl_spin_ = new QSpinBox();
    cache_ttl_spin_->setRange(1, 60);
    cache_ttl_spin_->setValue(5);
    cache_inner->addWidget(new QLabel("Cache TTL (seconds):"));
    cache_inner->addWidget(cache_ttl_spin_);
    cache_group->setLayout(cache_inner);
    cache_layout->addWidget(cache_group);
    cache_layout->addStretch();
    cache_widget->setLayout(cache_layout);
    tab_widget->addTab(cache_widget, "Cache");

    // Update intervals tab
    auto intervals_widget = new QWidget();
    auto intervals_layout = new QVBoxLayout();
    auto intervals_group = new QGroupBox("Update Intervals (ms)");
    auto intervals_inner = new QVBoxLayout();
    topic_update_interval_spin_ = new QSpinBox();
    topic_update_interval_spin_->setRange(500, 10000);
    topic_update_interval_spin_->setValue(3000);
    node_update_interval_spin_ = new QSpinBox();
    node_update_interval_spin_->setRange(500, 10000);
    node_update_interval_spin_->setValue(3000);
    metrics_update_interval_spin_ = new QSpinBox();
    metrics_update_interval_spin_->setRange(500, 10000);
    metrics_update_interval_spin_->setValue(1000);

    intervals_inner->addWidget(new QLabel("Topic Refresh:"));
    intervals_inner->addWidget(topic_update_interval_spin_);
    intervals_inner->addWidget(new QLabel("Node Refresh:"));
    intervals_inner->addWidget(node_update_interval_spin_);
    intervals_inner->addWidget(new QLabel("Metrics Refresh:"));
    intervals_inner->addWidget(metrics_update_interval_spin_);
    intervals_group->setLayout(intervals_inner);
    intervals_layout->addWidget(intervals_group);
    intervals_layout->addStretch();
    intervals_widget->setLayout(intervals_layout);
    tab_widget->addTab(intervals_widget, "Update Intervals");

    // Thread pool tab
    auto thread_widget = new QWidget();
    auto thread_layout = new QVBoxLayout();
    auto thread_group = new QGroupBox("Thread Pool");
    auto thread_inner = new QVBoxLayout();
    thread_pool_size_spin_ = new QSpinBox();
    thread_pool_size_spin_->setRange(1, 4);
    thread_pool_size_spin_->setValue(2);
    thread_inner->addWidget(new QLabel("Worker Threads:"));
    thread_inner->addWidget(thread_pool_size_spin_);
    thread_group->setLayout(thread_inner);
    thread_layout->addWidget(thread_group);
    thread_layout->addStretch();
    thread_widget->setLayout(thread_layout);
    tab_widget->addTab(thread_widget, "Performance");

    // Network settings tab
    auto network_widget = new QWidget();
    auto network_layout = new QVBoxLayout();
    auto network_group = new QGroupBox("Network Upload");
    auto network_inner = new QVBoxLayout();
    
    upload_endpoint_edit_ = new QLineEdit();
    api_key_edit_ = new QLineEdit();
    api_key_edit_->setEchoMode(QLineEdit::Password);
    chunk_size_spin_ = new QSpinBox();
    chunk_size_spin_->setRange(1, 100);
    chunk_size_spin_->setValue(5);
    max_concurrent_uploads_spin_ = new QSpinBox();
    max_concurrent_uploads_spin_->setRange(1, 10);
    max_concurrent_uploads_spin_->setValue(2);
    bandwidth_limit_spin_ = new QDoubleSpinBox();
    bandwidth_limit_spin_->setRange(0.0, 1000.0);
    bandwidth_limit_spin_->setValue(0.0);

    network_inner->addWidget(new QLabel("Upload Endpoint URL:"));
    network_inner->addWidget(upload_endpoint_edit_);
    network_inner->addWidget(new QLabel("API Key:"));
    network_inner->addWidget(api_key_edit_);
    network_inner->addWidget(new QLabel("Chunk Size (MB):"));
    network_inner->addWidget(chunk_size_spin_);
    network_inner->addWidget(new QLabel("Max Concurrent Uploads:"));
    network_inner->addWidget(max_concurrent_uploads_spin_);
    network_inner->addWidget(new QLabel("Bandwidth Limit (MB/s):"));
    network_inner->addWidget(bandwidth_limit_spin_);
    network_group->setLayout(network_inner);
    network_layout->addWidget(network_group);
    network_layout->addStretch();
    network_widget->setLayout(network_layout);
    tab_widget->addTab(network_widget, "Network");

    // Recording tab
    auto recording_widget = new QWidget();
    auto recording_layout = new QVBoxLayout();
    auto recording_group = new QGroupBox("Recording");
    auto recording_inner = new QVBoxLayout();
    recording_dir_edit_ = new QLineEdit();
    recording_inner->addWidget(new QLabel("Default Recording Directory:"));
    recording_inner->addWidget(recording_dir_edit_);
    recording_group->setLayout(recording_inner);
    recording_layout->addWidget(recording_group);
    recording_layout->addStretch();
    recording_widget->setLayout(recording_layout);
    tab_widget->addTab(recording_widget, "Recording");

    // UI tab
    auto ui_widget = new QWidget();
    auto ui_layout = new QVBoxLayout();
    auto ui_group = new QGroupBox("User Interface");
    auto ui_inner = new QVBoxLayout();
    dark_theme_checkbox_ = new QCheckBox("Dark Theme");
    debug_logging_checkbox_ = new QCheckBox("Debug Logging");
    ui_inner->addWidget(dark_theme_checkbox_);
    ui_inner->addWidget(debug_logging_checkbox_);
    ui_group->setLayout(ui_inner);
    ui_layout->addWidget(ui_group);
    ui_layout->addStretch();
    ui_widget->setLayout(ui_layout);
    tab_widget->addTab(ui_widget, "UI");

    main_layout->addWidget(tab_widget);

    // Buttons
    auto buttons_layout = new QHBoxLayout();
    apply_button_ = new QPushButton("Apply");
    reset_button_ = new QPushButton("Reset");
    ok_button_ = new QPushButton("OK");
    cancel_button_ = new QPushButton("Cancel");

    connect(apply_button_, &QPushButton::clicked, this, &SettingsDialog::on_apply_clicked);
    connect(reset_button_, &QPushButton::clicked, this, &SettingsDialog::on_reset_clicked);
    connect(ok_button_, &QPushButton::clicked, this, &SettingsDialog::accept);
    connect(cancel_button_, &QPushButton::clicked, this, &SettingsDialog::reject);

    buttons_layout->addWidget(apply_button_);
    buttons_layout->addWidget(reset_button_);
    buttons_layout->addStretch();
    buttons_layout->addWidget(ok_button_);
    buttons_layout->addWidget(cancel_button_);

    main_layout->addLayout(buttons_layout);
    setLayout(main_layout);
    resize(500, 600);
}

void SettingsDialog::load_settings_() {
    // Load from config.json in the config directory
    QFile config_file("config/config.json");
    
    if (!config_file.open(QIODevice::ReadOnly)) {
        std::cerr << "[SettingsDialog] Could not open config.json, using defaults" << std::endl;
        return;
    }
    
    QByteArray data = config_file.readAll();
    config_file.close();
    
    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (!doc.isObject()) {
        std::cerr << "[SettingsDialog] Invalid JSON in config.json" << std::endl;
        return;
    }
    
    QJsonObject obj = doc.object();
    
    // Load Cache settings
    if (obj.contains("cache")) {
        QJsonObject cache = obj["cache"].toObject();
        cache_ttl_spin_->setValue(cache["ttl_seconds"].toInt(5));
    }
    
    // Load Update Intervals
    if (obj.contains("update_intervals")) {
        QJsonObject intervals = obj["update_intervals"].toObject();
        topic_update_interval_spin_->setValue(intervals["topics_ms"].toInt(3000));
        node_update_interval_spin_->setValue(intervals["nodes_ms"].toInt(3000));
        metrics_update_interval_spin_->setValue(intervals["metrics_ms"].toInt(1000));
    }
    
    // Load Thread Pool settings
    if (obj.contains("thread_pool")) {
        QJsonObject thread_pool = obj["thread_pool"].toObject();
        thread_pool_size_spin_->setValue(thread_pool["num_threads"].toInt(2));
    }
    
    // Load Recording settings
    if (obj.contains("recording")) {
        QJsonObject recording = obj["recording"].toObject();
        recording_dir_edit_->setText(recording["default_directory"].toString("/tmp/ros2_bags"));
    }
    
    // Load Network settings
    if (obj.contains("network")) {
        QJsonObject network = obj["network"].toObject();
        upload_endpoint_edit_->setText(network["upload_endpoint"].toString("http://localhost:8080"));
        api_key_edit_->setText(network["api_key"].toString(""));
        chunk_size_spin_->setValue(network["chunk_size_mb"].toInt(5));
        max_concurrent_uploads_spin_->setValue(network["max_concurrent_uploads"].toInt(2));
        bandwidth_limit_spin_->setValue(network["bandwidth_limit_mbps"].toDouble(0.0));
    }
    
    // Load UI settings
    if (obj.contains("ui")) {
        QJsonObject ui = obj["ui"].toObject();
        dark_theme_checkbox_->setChecked(ui["dark_theme"].toBool(false));
        debug_logging_checkbox_->setChecked(ui["debug_logging"].toBool(false));
    }
    
    std::cerr << "[SettingsDialog] Loaded settings from config.json" << std::endl;
}

void SettingsDialog::save_settings_() {
    // Load existing config to preserve structure
    QFile config_file("config/config.json");
    QJsonObject obj;
    
    if (config_file.open(QIODevice::ReadOnly)) {
        QByteArray data = config_file.readAll();
        config_file.close();
        
        QJsonDocument doc = QJsonDocument::fromJson(data);
        if (doc.isObject()) {
            obj = doc.object();
        }
    }
    
    // Update Cache settings
    QJsonObject cache = obj["cache"].toObject();
    cache["ttl_seconds"] = cache_ttl_spin_->value();
    obj["cache"] = cache;
    
    // Update Update Intervals
    QJsonObject intervals = obj["update_intervals"].toObject();
    intervals["topics_ms"] = topic_update_interval_spin_->value();
    intervals["nodes_ms"] = node_update_interval_spin_->value();
    intervals["metrics_ms"] = metrics_update_interval_spin_->value();
    obj["update_intervals"] = intervals;
    
    // Update Thread Pool settings
    QJsonObject thread_pool = obj["thread_pool"].toObject();
    thread_pool["num_threads"] = thread_pool_size_spin_->value();
    obj["thread_pool"] = thread_pool;
    
    // Update Recording settings
    QJsonObject recording = obj["recording"].toObject();
    recording["default_directory"] = recording_dir_edit_->text();
    obj["recording"] = recording;
    
    // Update Network settings
    QJsonObject network = obj["network"].toObject();
    network["upload_endpoint"] = upload_endpoint_edit_->text();
    network["api_key"] = api_key_edit_->text();
    network["chunk_size_mb"] = chunk_size_spin_->value();
    network["max_concurrent_uploads"] = max_concurrent_uploads_spin_->value();
    network["bandwidth_limit_mbps"] = bandwidth_limit_spin_->value();
    obj["network"] = network;
    
    // Update UI settings
    QJsonObject ui = obj["ui"].toObject();
    ui["dark_theme"] = dark_theme_checkbox_->isChecked();
    ui["debug_logging"] = debug_logging_checkbox_->isChecked();
    obj["ui"] = ui;
    
    // Write to file
    if (config_file.open(QIODevice::WriteOnly)) {
        QJsonDocument doc(obj);
        config_file.write(doc.toJson());
        config_file.close();
        std::cerr << "[SettingsDialog] Saved settings to config.json" << std::endl;
    } else {
        std::cerr << "[SettingsDialog] Could not write to config.json" << std::endl;
    }
}

}  // namespace ros2_dashboard::gui
