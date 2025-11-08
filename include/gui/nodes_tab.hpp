/**
 * @file gui/nodes_tab.hpp
 * @brief ROS2 nodes discovery and visualization tab
 * @author Dashboard Team
 */

#pragma once

#include <QWidget>
#include <QTreeWidget>
#include <QPushButton>
#include <QLabel>
#include <memory>
#include "ros2_manager.hpp"
#include "async_worker.hpp"

namespace ros2_dashboard::gui {

/**
 * @class NodesTab
 * @brief Displays discovered nodes with connections
 */
class NodesTab : public QWidget {
    Q_OBJECT

public:
    explicit NodesTab(QWidget* parent = nullptr);
    ~NodesTab() override;

    void initialize(std::shared_ptr<ROS2Manager> ros2_manager,
                   std::shared_ptr<AsyncWorker> async_worker);

    void refresh_nodes();

signals:
    /**
     * @brief Signal emitted when nodes tree should be updated (thread-safe)
     */
    void on_nodes_updated(const std::vector<ros2_dashboard::NodeInfo>& nodes);

private slots:
    void on_refresh_button_clicked();
    void on_nodes_updated_slot(const std::vector<ros2_dashboard::NodeInfo>& nodes);

private:
    std::shared_ptr<ROS2Manager> ros2_manager_;
    std::shared_ptr<AsyncWorker> async_worker_;

    QTreeWidget* nodes_tree_;
    QLabel* total_nodes_label_;
    class QTimer* refresh_timer_;

    void create_ui_();
    void update_nodes_tree(const std::vector<ros2_dashboard::NodeInfo>& nodes);
};

}  // namespace ros2_dashboard::gui
