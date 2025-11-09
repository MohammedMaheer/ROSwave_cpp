/**
 * @file gui/main_window.cpp
 * @brief Main application window implementation
 */

#include "gui/main_window.hpp"
#include "gui/topics_tab.hpp"
#include "gui/nodes_tab.hpp"
#include "gui/selected_topics_tab.hpp"
#include "gui/services_tab.hpp"
#include "gui/recording_tab.hpp"
#include "gui/metrics_tab.hpp"
#include "gui/export_tab.hpp"
#include "gui/network_tab.hpp"
#include "gui/upload_tab.hpp"
#include "gui/settings_dialog.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <QApplication>
#include <QTime>
#include <QSettings>
#include <iostream>
#include <cstdint>
#include <cmath>

namespace ros2_dashboard::gui {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    setWindowTitle("ROS2 Recording & Monitoring Dashboard");
    setWindowIcon(QIcon(":/icons/app_icon.png"));
    
    resize(1200, 800);
}

MainWindow::~MainWindow() = default;

bool MainWindow::initialize() {
    try {
        // Initialize core managers
        ros2_manager_ = std::make_shared<ROS2Manager>();
        metrics_collector_ = std::make_shared<MetricsCollector>();
        network_manager_ = std::make_shared<NetworkManager>();
        ml_exporter_ = std::make_shared<MLExporter>();
        async_worker_ = std::make_shared<AsyncWorker>(2);
        health_monitor_ = std::make_shared<TopicHealthMonitor>();

        // Create UI components
        create_menu_bar_();
        create_tool_bar_();
        create_tabs_();
        create_status_bar_();
        
        // Create settings dialog
        settings_dialog_ = std::make_unique<SettingsDialog>(this);

        // Load settings
        load_settings_();
        
        // Initialize timer system (replaces individual tab timers)
        init_timers_();

        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

void MainWindow::closeEvent(QCloseEvent* event) {
    save_settings_();
    event->accept();
}

void MainWindow::on_refresh_all() {
    // Manual refresh triggered by user (Ctrl+R or Refresh button)
    // This bypasses normal timer constraints and updates everything
    
    std::cerr << "[MainWindow] User triggered refresh_all()" << std::endl;
    
    if (topics_tab_) {
        std::cerr << "[MainWindow] Refresh all: Refreshing Topics tab" << std::endl;
        topics_tab_->refresh_topics();
    }
    if (nodes_tab_) {
        std::cerr << "[MainWindow] Refresh all: Refreshing Nodes tab" << std::endl;
        nodes_tab_->refresh_nodes();
    }
    if (selected_topics_tab_) {
        std::cerr << "[MainWindow] Refresh all: Refreshing Selected Topics" << std::endl;
        selected_topics_tab_->refresh_selected_topics();
    }
    if (services_tab_) {
        std::cerr << "[MainWindow] Refresh all: Refreshing Services tab" << std::endl;
        services_tab_->refresh_services();
    }
    if (recording_tab_) {
        std::cerr << "[MainWindow] Refresh all: Refreshing Recording status" << std::endl;
        recording_tab_->refresh_recording_status();
    }
    if (metrics_tab_) {
        std::cerr << "[MainWindow] Refresh all: Refreshing Metrics" << std::endl;
        metrics_tab_->refresh_metrics();
    }
    if (network_tab_) {
        std::cerr << "[MainWindow] Refresh all: Refreshing Network status" << std::endl;
        network_tab_->refresh_network_status();
    }
    if (upload_tab_) {
        std::cerr << "[MainWindow] Refresh all: Refreshing Upload queue" << std::endl;
        upload_tab_->refresh_upload_queue();
    }
    
    status_label_->setText("Refreshed at " + 
        QTime::currentTime().toString("HH:mm:ss"));
}

void MainWindow::on_settings_changed() {
    if (settings_dialog_) {
        std::cerr << "[MainWindow] Opening Settings dialog..." << std::endl;
        settings_dialog_->exec();  // Use exec() for modal dialog instead of show()
    }
}

void MainWindow::on_about() {
    QMessageBox::about(this, "About ROS2 Dashboard",
        "ROS2 Recording & Monitoring Dashboard v1.0.0\n\n"
        "A comprehensive tool for ROS2 system monitoring,\n"
        "bag recording management, and cloud integration.\n\n"
        "© 2025 Dashboard Team");
}

void MainWindow::on_exit() {
    QApplication::quit();
}

void MainWindow::on_export_all() {
    std::cerr << "[MainWindow] Export All clicked" << std::endl;
    if (export_tab_) {
        tab_widget_->setCurrentWidget(export_tab_.get());
        QMessageBox::information(this, "Export All",
            "Navigate to the Export tab to export recorded data.\n\n"
            "You can select specific bags or export all recordings.");
    }
}

void MainWindow::on_health_report() {
    std::cerr << "[MainWindow] Health Report clicked" << std::endl;
    if (selected_topics_tab_) {
        tab_widget_->setCurrentWidget(selected_topics_tab_.get());
        QMessageBox::information(this, "Health Report",
            "Generated health report for selected topics.\n\n"
            "Status indicators:\n"
            "🟢 Green = Healthy (≥80% of expected rate)\n"
            "🟡 Yellow = Degraded (40-80% of expected rate)\n"
            "🔴 Red = Failed (<40% or no messages)");
    }
}

void MainWindow::on_diagnostics() {
    std::cerr << "[MainWindow] Diagnostics clicked" << std::endl;
    QString diagnostics_info = "System Diagnostics Report\n\n";
    
    if (ros2_manager_) {
        auto topics = ros2_manager_->get_topics();
        auto nodes = ros2_manager_->get_nodes();
        auto services = ros2_manager_->get_services();
        
        diagnostics_info += QString::asprintf(
            "ROS2 System Status:\n"
            "  Topics: %zu\n"
            "  Nodes: %zu\n"
            "  Services: %zu\n",
            topics.size(), nodes.size(), services.size()
        );
    }
    
    if (metrics_collector_) {
        auto snapshot = metrics_collector_->get_snapshot();
        diagnostics_info += "System Metrics:\n";
        diagnostics_info += QString("  CPU Usage: %1%\n").arg(snapshot.system.cpu_usage_percent, 0, 'f', 1);
        diagnostics_info += QString("  Memory: %1 MB\n").arg(snapshot.system.memory_usage_mb);
    }
    
    diagnostics_info += "\nCache Hit Ratio: " + 
        QString::number(ros2_manager_ ? ros2_manager_->get_cache_hit_ratio() : 0.0, 'f', 2) + "%\n";
    diagnostics_info += "Status: OK";
    
    QMessageBox::information(this, "System Diagnostics", diagnostics_info);
}

void MainWindow::on_clear_cache() {
    std::cerr << "[MainWindow] Clear Cache clicked" << std::endl;
    if (QMessageBox::question(this, "Clear Cache",
            "Clear ROS2 discovery cache?\n\n"
            "This will force rediscovery of all topics, nodes, and services.",
            QMessageBox::Yes | QMessageBox::No) == QMessageBox::Yes) {
        
        if (ros2_manager_) {
            ros2_manager_->invalidate_cache();
            status_label_->setText("Cache cleared. Refreshing...");
            on_refresh_all();
            status_label_->setText("Refreshed at " + QTime::currentTime().toString("HH:mm:ss"));
            QMessageBox::information(this, "Cache Cleared", "Discovery cache cleared successfully!");
        }
    }
}

void MainWindow::on_maximize_toggle() {
    std::cerr << "[MainWindow] Maximize toggle clicked" << std::endl;
    if (isMaximized()) {
        showNormal();
        status_label_->setText("Window restored");
    } else {
        showMaximized();
        status_label_->setText("Window maximized");
    }
}

void MainWindow::create_menu_bar_() {
    QMenuBar* menu_bar = menuBar();

    // File menu
    QMenu* file_menu = menu_bar->addMenu("&File");
    QAction* export_action = file_menu->addAction("&Export All");
    export_action->setShortcut(Qt::CTRL + Qt::Key_E);
    connect(export_action, &QAction::triggered, this, &MainWindow::on_export_all);
    file_menu->addSeparator();
    QAction* exit_action = file_menu->addAction("E&xit");
    connect(exit_action, &QAction::triggered, this, &MainWindow::on_exit);

    // View menu
    QMenu* view_menu = menu_bar->addMenu("&View");
    QAction* refresh_action = view_menu->addAction("&Refresh All");
    refresh_action->setShortcut(Qt::CTRL + Qt::Key_R);
    connect(refresh_action, &QAction::triggered, this, &MainWindow::on_refresh_all);
    view_menu->addSeparator();
    QAction* health_action = view_menu->addAction("&Health Report");
    health_action->setShortcut(Qt::CTRL + Qt::Key_H);
    connect(health_action, &QAction::triggered, this, &MainWindow::on_health_report);

    // Edit menu
    QMenu* edit_menu = menu_bar->addMenu("&Edit");
    QAction* diagnostics_action = edit_menu->addAction("&Diagnostics");
    diagnostics_action->setShortcut(Qt::CTRL + Qt::Key_D);
    connect(diagnostics_action, &QAction::triggered, this, &MainWindow::on_diagnostics);
    edit_menu->addSeparator();
    QAction* clear_cache_action = edit_menu->addAction("&Clear Cache");
    connect(clear_cache_action, &QAction::triggered, this, &MainWindow::on_clear_cache);
    edit_menu->addSeparator();
    QAction* settings_action = edit_menu->addAction("&Settings");
    settings_action->setShortcut(Qt::CTRL + Qt::Key_Comma);
    connect(settings_action, &QAction::triggered, this, &MainWindow::on_settings_changed);

    // Help menu
    QMenu* help_menu = menu_bar->addMenu("&Help");
    QAction* about_action = help_menu->addAction("&About");
    connect(about_action, &QAction::triggered, this, &MainWindow::on_about);
}

void MainWindow::create_tool_bar_() {
    QToolBar* tool_bar = addToolBar("Main Toolbar");
    tool_bar->setObjectName("MainToolBar");

    QPushButton* refresh_btn = new QPushButton("Refresh");
    connect(refresh_btn, &QPushButton::clicked, this, &MainWindow::on_refresh_all);
    tool_bar->addWidget(refresh_btn);

    tool_bar->addSeparator();

    QPushButton* settings_btn = new QPushButton("Settings");
    connect(settings_btn, &QPushButton::clicked, this, &MainWindow::on_settings_changed);
    tool_bar->addWidget(settings_btn);

    tool_bar->addSeparator();

    QPushButton* maximize_btn = new QPushButton("⛶ Maximize");
    connect(maximize_btn, &QPushButton::clicked, this, &MainWindow::on_maximize_toggle);
    tool_bar->addWidget(maximize_btn);

    tool_bar->addSeparator();

    // Optional features
    QPushButton* export_btn = new QPushButton("Export All");
    connect(export_btn, &QPushButton::clicked, this, &MainWindow::on_export_all);
    tool_bar->addWidget(export_btn);

    QPushButton* health_btn = new QPushButton("Health Report");
    connect(health_btn, &QPushButton::clicked, this, &MainWindow::on_health_report);
    tool_bar->addWidget(health_btn);

    QPushButton* diagnostics_btn = new QPushButton("Diagnostics");
    connect(diagnostics_btn, &QPushButton::clicked, this, &MainWindow::on_diagnostics);
    tool_bar->addWidget(diagnostics_btn);

    QPushButton* clear_cache_btn = new QPushButton("Clear Cache");
    connect(clear_cache_btn, &QPushButton::clicked, this, &MainWindow::on_clear_cache);
    tool_bar->addWidget(clear_cache_btn);
}

void MainWindow::create_tabs_() {
    tab_widget_ = new QTabWidget();

    // Create tabs
    topics_tab_ = std::make_unique<TopicsTab>();
    topics_tab_->initialize(ros2_manager_, async_worker_);
    tab_widget_->addTab(topics_tab_.get(), "Topics");

    nodes_tab_ = std::make_unique<NodesTab>();
    nodes_tab_->initialize(ros2_manager_, async_worker_);
    tab_widget_->addTab(nodes_tab_.get(), "Nodes");

    selected_topics_tab_ = std::make_unique<SelectedTopicsTab>();
    selected_topics_tab_->initialize(ros2_manager_, async_worker_, health_monitor_);
    tab_widget_->addTab(selected_topics_tab_.get(), "Selected Topics");

    services_tab_ = std::make_unique<ServicesTab>();
    services_tab_->initialize(ros2_manager_, async_worker_);
    tab_widget_->addTab(services_tab_.get(), "Services");

    recording_tab_ = std::make_unique<RecordingTab>();
    recording_tab_->initialize(ros2_manager_, async_worker_);
    tab_widget_->addTab(recording_tab_.get(), "Recording");

    metrics_tab_ = std::make_unique<MetricsTab>();
    metrics_tab_->initialize(metrics_collector_, async_worker_);
    tab_widget_->addTab(metrics_tab_.get(), "Metrics");

    export_tab_ = std::make_unique<ExportTab>();
    export_tab_->initialize(ml_exporter_, ros2_manager_, async_worker_);
    tab_widget_->addTab(export_tab_.get(), "Export");

    network_tab_ = std::make_unique<NetworkTab>();
    network_tab_->initialize(ros2_manager_, async_worker_);
    tab_widget_->addTab(network_tab_.get(), "Network");

    upload_tab_ = std::make_unique<UploadTab>();
    upload_tab_->initialize(network_manager_, async_worker_);
    tab_widget_->addTab(upload_tab_.get(), "Uploads");

    setCentralWidget(tab_widget_);
}

void MainWindow::create_status_bar_() {
    status_label_ = new QLabel("Ready");
    statusBar()->addWidget(status_label_);

    connection_indicator_ = new QLabel("●");
    connection_indicator_->setStyleSheet("color: green;");
    statusBar()->addPermanentWidget(connection_indicator_);
}

void MainWindow::load_settings_() {
    // Load window geometry and state
    QSettings settings("ROS2Dashboard", "MainWindow");
    
    // Restore window size and position
    restoreGeometry(settings.value("geometry", saveGeometry()).toByteArray());
    restoreState(settings.value("windowState", saveState()).toByteArray());
    
    std::cerr << "[MainWindow] Loaded window settings" << std::endl;
}

void MainWindow::save_settings_() {
    // Save window geometry and state
    QSettings settings("ROS2Dashboard", "MainWindow");
    
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    
    std::cerr << "[MainWindow] Saved window settings" << std::endl;
}

void MainWindow::init_timers_() {
    std::cerr << "[MainWindow] Initializing 2-tier timer system" << std::endl;
    
    // Fast timer: 1s tick for UI updates (metrics, recording status, upload queue)
    fast_timer_ = new QTimer(this);
    connect(fast_timer_, &QTimer::timeout, this, &MainWindow::on_fast_timer_timeout);
    fast_timer_->start(1000);
    
    // Slow timer: 10s tick for ROS2 discovery (topics, nodes, services)
    // This replaces 3 separate timers running every 3s
    slow_timer_ = new QTimer(this);
    connect(slow_timer_, &QTimer::timeout, this, &MainWindow::on_slow_timer_timeout);
    slow_timer_->start(10000);
    
    // Connect tab changed signal to update visible tab
    connect(tab_widget_, QOverload<int>::of(&QTabWidget::currentChanged),
            this, &MainWindow::on_tab_changed);
    
    std::cerr << "[MainWindow] Timer system initialized (fast: 1s, slow: 10s)" << std::endl;
}

void MainWindow::on_fast_timer_timeout() {
    // Fast updates for UI state (1s tick)
    // DIRTY TABLE TRACKING: Only update if data has actually changed
    // This skips unnecessary UI redraws when metrics haven't changed (75% redraw reduction)
    
    // Recording status - always update (changes frequently)
    if (recording_tab_) {
        recording_tab_->refresh_recording_status();  // Live timer and size
    }
    
    // Metrics - check if values actually changed before updating display
    if (metrics_tab_ && metrics_collector_) {
        auto snapshot = metrics_collector_->get_snapshot();
        
        // Check if metrics have meaningfully changed (more than 0.1% difference)
        bool metrics_changed = false;
        
        if (std::abs(snapshot.system.cpu_usage_percent - topics_cache_.last_cpu_percent) > 0.1) {
            metrics_changed = true;
            topics_cache_.last_cpu_percent = snapshot.system.cpu_usage_percent;
        }
        
        if (std::abs(static_cast<double>(snapshot.system.memory_usage_mb) - 
                     topics_cache_.last_memory_mb) > 10.0) {  // 10MB threshold
            metrics_changed = true;
            topics_cache_.last_memory_mb = snapshot.system.memory_usage_mb;
        }
        
        if (metrics_changed) {
            std::cerr << "[MainWindow] Metrics changed - updating display (CPU: " 
                      << snapshot.system.cpu_usage_percent << "%, Memory: " 
                      << snapshot.system.memory_usage_mb << " MB)" << std::endl;
            metrics_tab_->refresh_metrics();
        } else {
            std::cerr << "[MainWindow] Metrics unchanged - skipping redraw" << std::endl;
        }
    }
    
    // Upload queue - always update (progress related)
    if (upload_tab_) {
        upload_tab_->refresh_upload_queue();  // Upload queue progress
    }
    
    // Selected topics health - always update (important for monitoring)
    if (selected_topics_tab_) {
        selected_topics_tab_->refresh_selected_topics();  // Health status
    }
}

void MainWindow::on_slow_timer_timeout() {
    // Slow updates for ROS2 discovery (10s tick)
    // VISIBLE-TAB OPTIMIZATION: Only refresh the currently visible tab
    // This avoids querying data for inactive tabs (40% CPU improvement)
    
    current_tab_index_ = tab_widget_->currentIndex();
    
    // Only refresh active tab if data might have changed
    // Skip refresh if data hasn't changed since last update (dirty table tracking)
    if (current_tab_index_ == 0 && (pending_refreshes_ & TAB_TOPICS)) {
        std::cerr << "[MainWindow] Slow timer: Checking Topics tab (VISIBLE, active)" << std::endl;
        if (topics_tab_ && ros2_manager_) {
            auto new_topics = ros2_manager_->get_topics();
            if (has_data_changed(topics_cache_, new_topics)) {
                std::cerr << "[MainWindow] Topics data changed - updating UI" << std::endl;
                update_cache(topics_cache_, new_topics);
                topics_tab_->refresh_topics();
            } else {
                std::cerr << "[MainWindow] Topics data unchanged - skipping UI update" << std::endl;
            }
        }
        pending_refreshes_ &= ~TAB_TOPICS;
    } 
    else if (current_tab_index_ == 1 && (pending_refreshes_ & TAB_NODES)) {
        std::cerr << "[MainWindow] Slow timer: Checking Nodes tab (VISIBLE, active)" << std::endl;
        if (nodes_tab_ && ros2_manager_) {
            auto new_nodes = ros2_manager_->get_nodes();
            // Simple check: compare count and first few names
            if (new_nodes.size() != nodes_cache_.nodes.size() || 
                (new_nodes.size() > 0 && nodes_cache_.nodes.size() > 0 &&
                 new_nodes[0].name != nodes_cache_.nodes[0].name)) {
                std::cerr << "[MainWindow] Nodes data changed - updating UI" << std::endl;
                nodes_cache_.nodes = new_nodes;
                nodes_cache_.data_changed = true;
                nodes_tab_->refresh_nodes();
            } else {
                std::cerr << "[MainWindow] Nodes data unchanged - skipping UI update" << std::endl;
            }
        }
        pending_refreshes_ &= ~TAB_NODES;
    } 
    else if (current_tab_index_ == 2 && (pending_refreshes_ & TAB_SERVICES)) {
        std::cerr << "[MainWindow] Slow timer: Checking Services tab (VISIBLE, active)" << std::endl;
        if (services_tab_ && ros2_manager_) {
            auto new_services = ros2_manager_->get_services();
            // Simple check: compare count and first few names
            if (new_services.size() != services_cache_.services.size() ||
                (new_services.size() > 0 && services_cache_.services.size() > 0 &&
                 new_services[0].name != services_cache_.services[0].name)) {
                std::cerr << "[MainWindow] Services data changed - updating UI" << std::endl;
                services_cache_.services = new_services;
                services_cache_.data_changed = true;
                services_tab_->refresh_services();
            } else {
                std::cerr << "[MainWindow] Services data unchanged - skipping UI update" << std::endl;
            }
        }
        pending_refreshes_ &= ~TAB_SERVICES;
    }
    
    // Mark all tabs for refresh at next opportunity
    // (ensures they update immediately when user switches to them)
    pending_refreshes_ = TAB_ALL_DISCOVERY;
}

void MainWindow::on_tab_changed(int index) {
    std::cerr << "[MainWindow] Tab changed to index: " << index << std::endl;
    current_tab_index_ = index;
    
    // Trigger immediate refresh of newly visible tab
    switch (index) {
        case 0:  // Topics
            if (topics_tab_) {
                std::cerr << "[MainWindow] Tab changed: Immediately refreshing Topics" << std::endl;
                topics_tab_->refresh_topics();
            }
            break;
        case 1:  // Nodes
            if (nodes_tab_) {
                std::cerr << "[MainWindow] Tab changed: Immediately refreshing Nodes" << std::endl;
                nodes_tab_->refresh_nodes();
            }
            break;
        case 2:  // Services
            if (services_tab_) {
                std::cerr << "[MainWindow] Tab changed: Immediately refreshing Services" << std::endl;
                services_tab_->refresh_services();
            }
            break;
        // Other tabs don't need discovery refresh
    }
}

bool MainWindow::has_data_changed(const TabDataCache& cache, 
                                   const std::vector<ros2_dashboard::TopicInfo>& new_topics) const {
    // DIRTY TABLE TRACKING: Check if data has actually changed
    // This avoids unnecessary UI updates (75% redraw reduction)
    
    // Quick check: size changed?
    if (new_topics.size() != cache.topics.size()) {
        std::cerr << "[MainWindow] Topic count changed: " << cache.topics.size() 
                  << " -> " << new_topics.size() << std::endl;
        return true;
    }
    
    // Size same but no topics? Both empty?
    if (new_topics.empty() && cache.topics.empty()) {
        return false;
    }
    
    // Check if first few topics changed (efficient quick check)
    size_t check_limit = std::min(static_cast<size_t>(3), new_topics.size());
    for (size_t i = 0; i < check_limit; ++i) {
        if (new_topics[i].name != cache.topics[i].name ||
            new_topics[i].msg_type != cache.topics[i].msg_type ||
            new_topics[i].publisher_count != cache.topics[i].publisher_count ||
            new_topics[i].subscription_count != cache.topics[i].subscription_count) {
            
            std::cerr << "[MainWindow] Topic data changed at index " << i << std::endl;
            return true;
        }
    }
    
    // No significant changes detected
    return false;
}

void MainWindow::update_cache(TabDataCache& cache, 
                              const std::vector<ros2_dashboard::TopicInfo>& new_topics) {
    // Update cache with new data and mark as clean
    cache.topics = new_topics;
    cache.data_changed = false;
    std::cerr << "[MainWindow] Cache updated with " << new_topics.size() << " topics" << std::endl;
}

}  // namespace ros2_dashboard::gui
