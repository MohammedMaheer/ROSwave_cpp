/**
 * @file gui/topics_tab.hpp
 * @brief Topics discovery and message preview tab
 * @author Dashboard Team
 */

#pragma once

#include <QWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QPushButton>
#include <QLabel>
#include <memory>
#include "ros2_manager.hpp"
#include "async_worker.hpp"

namespace ros2_dashboard::gui {

/**
 * @class TopicsTab
 * @brief Displays discovered topics with metadata and message preview
 */
class TopicsTab : public QWidget {
    Q_OBJECT

public:
    explicit TopicsTab(QWidget* parent = nullptr);
    ~TopicsTab() override;

    /**
     * @brief Initialize with core managers
     */
    void initialize(std::shared_ptr<ROS2Manager> ros2_manager,
                   std::shared_ptr<AsyncWorker> async_worker);

    /**
     * @brief Refresh topic list
     */
    void refresh_topics();

signals:
    /**
     * @brief Signal emitted when topic table should be updated (thread-safe)
     */
    void on_topics_updated(const std::vector<ros2_dashboard::TopicInfo>& topics);
    
    /**
     * @brief Signal emitted when message preview should be updated (thread-safe)
     */
    void on_message_updated(const QString& message);

private slots:
    void on_topic_selected(int row, int column);
    void on_refresh_button_clicked();
    void on_topics_updated_slot(const std::vector<ros2_dashboard::TopicInfo>& topics);
    void on_message_updated_slot(const QString& message);

private:
    std::shared_ptr<ROS2Manager> ros2_manager_;
    std::shared_ptr<AsyncWorker> async_worker_;

    QTableWidget* topics_table_;
    QTextEdit* message_preview_;
    QPushButton* refresh_button_;
    QLabel* total_topics_label_;
    class QTimer* refresh_timer_;

    void create_ui_();
    void update_topics_table(const std::vector<ros2_dashboard::TopicInfo>& topics);
};

}  // namespace ros2_dashboard::gui
