/**
 * @file gui/upload_tab.hpp
 * @brief Upload queue and cloud integration tab
 * @author Dashboard Team
 */

#pragma once

#include <QWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QProgressBar>
#include <QLabel>
#include <memory>
#include "network_manager.hpp"
#include "async_worker.hpp"

namespace ros2_dashboard::gui {

/**
 * @class UploadTab
 * @brief Manages upload queue and cloud synchronization
 */
class UploadTab : public QWidget {
    Q_OBJECT

public:
    explicit UploadTab(QWidget* parent = nullptr);
    ~UploadTab() override;

    void initialize(std::shared_ptr<NetworkManager> network_manager,
                   std::shared_ptr<AsyncWorker> async_worker);

    void refresh_upload_queue();

private slots:
    void on_queue_item_selected(int row, int column);
    void on_retry_clicked();
    void on_cancel_clicked();
    void on_clear_completed_clicked();
    void on_refresh_clicked();

private:
    std::shared_ptr<NetworkManager> network_manager_;
    std::shared_ptr<AsyncWorker> async_worker_;

    QTableWidget* upload_queue_table_;
    QTableWidget* upload_history_table_;
    QProgressBar* selected_upload_progress_;
    QLabel* queue_stats_label_;
    QPushButton* retry_button_;
    QPushButton* cancel_button_;
    QPushButton* clear_button_;
    QPushButton* refresh_button_;

    void create_ui_();
    void update_queue_display_();
};

}  // namespace ros2_dashboard::gui
