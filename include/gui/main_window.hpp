/**
 * @file gui/main_window.hpp
 * @brief Main application window with tab-based interface
 * @author Dashboard Team
 */

#pragma once

#include <QMainWindow>
#include <QTabWidget>
#include <QLabel>
#include <QTimer>
#include <memory>

#include "ros2_manager.hpp"
#include "metrics_collector.hpp"
#include "network_manager.hpp"
#include "ml_exporter.hpp"
#include "async_worker.hpp"
#include "topic_monitor.hpp"

namespace ros2_dashboard::gui {

class TopicsTab;
class NodesTab;
class SelectedTopicsTab;
class ServicesTab;
class RecordingTab;
class MetricsTab;
class ExportTab;
class NetworkTab;
class UploadTab;
class SettingsDialog;

/**
 * @class MainWindow
 * @brief Main application window
 * 
 * Coordinates all tabs and manages core component lifecycle.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    /**
     * @brief Initialize application components
     * @return true if initialization succeeded
     */
    bool initialize();

protected:
    void closeEvent(QCloseEvent* event) override;

private slots:
    void on_refresh_all();
    void on_settings_changed();
    void on_about();
    void on_exit();
    void on_export_all();
    void on_health_report();
    void on_diagnostics();
    void on_clear_cache();
    void on_fast_timer_timeout();
    void on_slow_timer_timeout();
    void on_tab_changed(int index);

private:
    // UI components
    QTabWidget* tab_widget_;
    QLabel* status_label_;
    QLabel* connection_indicator_;

    // 2-tier timer system for optimized refreshing
    QTimer* fast_timer_;     // 1s tick for UI updates (metrics, recording status)
    QTimer* slow_timer_;     // 10s tick for ROS2 discovery (topics, nodes, services)
    
    // Tab refresh flags (bitwise)
    enum TabRefreshFlag {
        TAB_NONE = 0,
        TAB_TOPICS = 1 << 0,
        TAB_NODES = 1 << 1,
        TAB_SERVICES = 1 << 2,
        TAB_SELECTED = 1 << 3,
        TAB_ALL_DISCOVERY = TAB_TOPICS | TAB_NODES | TAB_SERVICES
    };
    int pending_refreshes_ = TAB_ALL_DISCOVERY;  // Start with all tabs needing refresh
    int current_tab_index_ = 0;
    
    // Visible-tab optimization: only refresh currently active tabs
    // Dirty table tracking: skip UI updates if data unchanged
    struct TabDataCache {
        std::vector<ros2_dashboard::TopicInfo> topics;
        std::vector<ros2_dashboard::NodeInfo> nodes;
        std::vector<ros2_dashboard::ServiceInfo> services;
        std::string recording_status;
        uint64_t file_size_bytes = 0;
        double last_cpu_percent = -1.0;
        double last_memory_mb = -1.0;
        bool data_changed = true;  // Flag to track if refresh is needed
    };
    
    TabDataCache topics_cache_;
    TabDataCache nodes_cache_;
    TabDataCache services_cache_;

    // Core managers
    std::shared_ptr<ROS2Manager> ros2_manager_;
    std::shared_ptr<MetricsCollector> metrics_collector_;
    std::shared_ptr<NetworkManager> network_manager_;
    std::shared_ptr<MLExporter> ml_exporter_;
    std::shared_ptr<AsyncWorker> async_worker_;
    std::shared_ptr<TopicHealthMonitor> health_monitor_;

    // Tab views
    std::unique_ptr<TopicsTab> topics_tab_;
    std::unique_ptr<NodesTab> nodes_tab_;
    std::unique_ptr<SelectedTopicsTab> selected_topics_tab_;
    std::unique_ptr<ServicesTab> services_tab_;
    std::unique_ptr<RecordingTab> recording_tab_;
    std::unique_ptr<MetricsTab> metrics_tab_;
    std::unique_ptr<ExportTab> export_tab_;
    std::unique_ptr<NetworkTab> network_tab_;
    std::unique_ptr<UploadTab> upload_tab_;
    std::unique_ptr<SettingsDialog> settings_dialog_;

    void create_menu_bar_();
    void create_tool_bar_();
    void create_tabs_();
    void create_status_bar_();
    void load_settings_();
    void save_settings_();
    void init_timers_();
    
    // Optimization helpers
    bool has_data_changed(const TabDataCache& cache, 
                          const std::vector<ros2_dashboard::TopicInfo>& new_topics) const;
    void update_cache(TabDataCache& cache, 
                      const std::vector<ros2_dashboard::TopicInfo>& new_topics);
};

}  // namespace ros2_dashboard::gui
