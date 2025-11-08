/**
 * @file gui/recording_tab.hpp
 * @brief Recording control and bag management tab
 * @author Dashboard Team
 */

#pragma once

#include <QWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QCheckBox>
#include <QSpinBox>
#include <QComboBox>
#include <QLabel>
#include <QTimer>
#include <memory>
#include "ros2_manager.hpp"
#include "async_worker.hpp"

namespace ros2_dashboard::gui {

/**
 * @class RecordingTab
 * @brief Manages bag recording and displays bag history
 */
class RecordingTab : public QWidget {
    Q_OBJECT

public:
    explicit RecordingTab(QWidget* parent = nullptr);
    ~RecordingTab() override;

    void initialize(std::shared_ptr<ROS2Manager> ros2_manager,
                   std::shared_ptr<AsyncWorker> async_worker);

    void refresh_recording_status();
    void refresh_bag_list();

signals:
    void on_recording_status_updated(bool is_recording);

private slots:
    void on_start_recording_clicked();
    void on_stop_recording_clicked();
    void on_browse_directory_clicked();
    void on_refresh_bags_clicked();
    void on_bag_selected(int row, int column);
    void on_open_file_manager_clicked();
    void on_play_in_rviz_clicked();

private:
    std::shared_ptr<ROS2Manager> ros2_manager_;
    std::shared_ptr<AsyncWorker> async_worker_;

    // Recording controls
    QPushButton* start_button_;
    QPushButton* stop_button_;
    QLineEdit* output_dir_edit_;
    QPushButton* browse_button_;
    QComboBox* compression_combo_;
    QLabel* status_label_;
    QLabel* elapsed_time_label_;
    QLabel* file_size_label_;

    // Bag list
    QTableWidget* bags_table_;
    QPushButton* refresh_bags_button_;
    QPushButton* open_file_manager_button_;
    QPushButton* play_rviz_button_;

    // Auto-refresh timer
    QTimer* status_refresh_timer_;
    QTimer* bag_list_refresh_timer_;

    void create_ui_();
    void update_recording_controls_();
    void update_bag_list_();
};

}  // namespace ros2_dashboard::gui
