/**
 * @file gui/network_tab.hpp
 * @brief Multicast robot discovery and network status tab
 * @author Dashboard Team
 */

#pragma once

#include <QWidget>
#include <QTableWidget>
#include <QPushButton>
#include <QLabel>
#include <memory>
#include "ros2_manager.hpp"
#include "async_worker.hpp"

namespace ros2_dashboard::gui {

/**
 * @class NetworkTab
 * @brief Displays discovered robots and network status
 */
class NetworkTab : public QWidget {
    Q_OBJECT

public:
    explicit NetworkTab(QWidget* parent = nullptr);
    ~NetworkTab() override;

    void initialize(std::shared_ptr<ROS2Manager> ros2_manager,
                   std::shared_ptr<AsyncWorker> async_worker);

    void refresh_network_status();

private slots:
    void on_discover_robots_clicked();
    void on_refresh_status_clicked();

private:
    std::shared_ptr<ROS2Manager> ros2_manager_;
    std::shared_ptr<AsyncWorker> async_worker_;

    QTableWidget* robots_table_;
    QLabel* network_status_label_;
    QPushButton* discover_button_;
    QPushButton* refresh_button_;

    void create_ui_();
};

}  // namespace ros2_dashboard::gui
