/**
 * @file gui/selected_topics_tab.hpp
 * @brief Selected topics health monitoring with auto-discovery
 * @author Dashboard Team
 */

#pragma once

#include <QWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QComboBox>
#include <QSpinBox>
#include <QLabel>
#include <QListWidget>
#include <memory>
#include <map>
#include "ros2_manager.hpp"
#include "async_worker.hpp"
#include "topic_monitor.hpp"

namespace ros2_dashboard::gui {

/**
 * @class SelectedTopicsTab
 * @brief Monitor selected topics with health indicators and rate tracking
 * 
 * Features:
 * - Auto-discovery of available topics
 * - Add/remove topics from monitoring list
 * - Real-time message rate display
 * - Health status with red/green indicators
 * - Alert system for slow or failing message transfers
 * - Threshold configuration per topic
 */
class SelectedTopicsTab : public QWidget {
    Q_OBJECT

public:
    explicit SelectedTopicsTab(QWidget* parent = nullptr);
    ~SelectedTopicsTab() override;

    /**
     * @brief Initialize with core managers
     */
    void initialize(std::shared_ptr<ROS2Manager> ros2_manager,
                   std::shared_ptr<AsyncWorker> async_worker,
                   std::shared_ptr<TopicHealthMonitor> health_monitor);

    /**
     * @brief Refresh monitored topics list and health status
     */
    void refresh_monitored_topics();
    
    /**
     * @brief Refresh selected topics (alias for compatibility)
     */
    void refresh_selected_topics() { refresh_monitored_topics(); }

    /**
     * @brief Load monitored topics from settings
     */
    void load_monitored_topics_from_settings();

    /**
     * @brief Save monitored topics to settings
     */
    void save_monitored_topics_to_settings();

private slots:
    void on_auto_discover_clicked();
    void on_add_topic_clicked();
    void on_remove_selected_clicked();
    void on_clear_all_clicked();
    void on_refresh_health_clicked();
    void on_topic_selected(int row, int column);
    void on_threshold_changed(int value);
    void on_update_timer_timeout();

private:
    // Managers
    std::shared_ptr<ROS2Manager> ros2_manager_;
    std::shared_ptr<AsyncWorker> async_worker_;
    std::shared_ptr<TopicHealthMonitor> health_monitor_;

    // UI Components
    QTableWidget* monitored_topics_table_;
    QPushButton* auto_discover_button_;
    QPushButton* add_topic_button_;
    QPushButton* remove_button_;
    QPushButton* clear_button_;
    QPushButton* refresh_button_;
    QComboBox* available_topics_combo_;
    QListWidget* available_topics_list_;  // For multi-select checkboxes
    QLabel* discovery_status_label_;
    QPushButton* add_selected_topics_btn_;  // Bulk add from checkbox list
    
    // Status counts
    QLabel* total_topics_count_;
    QLabel* healthy_count_;
    QLabel* degraded_count_;
    QLabel* failed_count_;
    QLabel* recording_count_;
    
    // Timer
    QTimer* update_timer_;

    // Internal methods
    void create_ui_();
    void populate_available_topics_combo_();
};

}  // namespace ros2_dashboard::gui

