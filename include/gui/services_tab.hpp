/**
 * @file gui/services_tab.hpp
 * @brief ROS2 services discovery tab
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
 * @class ServicesTab
 * @brief Displays discovered services
 */
class ServicesTab : public QWidget {
    Q_OBJECT

public:
    explicit ServicesTab(QWidget* parent = nullptr);
    ~ServicesTab() override;

    void initialize(std::shared_ptr<ROS2Manager> ros2_manager,
                   std::shared_ptr<AsyncWorker> async_worker);

    void refresh_services();

signals:
    /**
     * @brief Signal emitted when services list should be updated (thread-safe)
     */
    void on_services_updated(const std::vector<ros2_dashboard::ServiceInfo>& services);

private slots:
    void on_refresh_button_clicked();
    void on_services_updated_slot(const std::vector<ros2_dashboard::ServiceInfo>& services);

private:
    std::shared_ptr<ROS2Manager> ros2_manager_;
    std::shared_ptr<AsyncWorker> async_worker_;

    QTableWidget* services_table_;
    QPushButton* refresh_button_;
    QLabel* total_services_label_;
    class QTimer* refresh_timer_;

    void create_ui_();
    void update_services_table(const std::vector<ros2_dashboard::ServiceInfo>& services);
};

}  // namespace ros2_dashboard::gui
