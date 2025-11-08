/**
 * @file gui/network_tab.cpp
 */

#include "gui/network_tab.hpp"
#include "ros2_manager.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTableWidget>
#include <QTime>
#include <QColor>

namespace ros2_dashboard::gui {

NetworkTab::NetworkTab(QWidget* parent) : QWidget(parent) { create_ui_(); }
NetworkTab::~NetworkTab() = default;

void NetworkTab::initialize(std::shared_ptr<ROS2Manager> ros2_manager,
                           std::shared_ptr<AsyncWorker> async_worker) {
    ros2_manager_ = ros2_manager;
    async_worker_ = async_worker;
}

void NetworkTab::refresh_network_status() {
    if (ros2_manager_) {
        // NOTE: Calling directly in main thread instead of async_worker
        // because updating Qt widgets from worker threads causes crashes
        // Get system network status
        auto nodes = ros2_manager_->get_nodes();
        
        // Clear current entries
        robots_table_->setRowCount(0);
        
        if (nodes.empty()) {
            network_status_label_->setText("Status: No ROS2 nodes discovered - network may be isolated");
            robots_table_->setRowCount(1);
            auto item = new QTableWidgetItem("(No nodes)");
            item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
            robots_table_->setItem(0, 0, item);
        } else {
            network_status_label_->setText(QString("Status: Found %1 ROS2 nodes").arg(nodes.size()));
            
            // Populate robot table with discovered nodes
            int row_idx = 0;
            for (const auto& node : nodes) {
                robots_table_->insertRow(row_idx);
                
                // Node name
                auto name_item = new QTableWidgetItem(QString::fromStdString(node.name));
                robots_table_->setItem(row_idx, 0, name_item);
                
                // IP Address (placeholder - would need DDS discovery plugin)
                auto ip_item = new QTableWidgetItem("Local");
                robots_table_->setItem(row_idx, 1, ip_item);
                
                // Status (Active if discovered)
                auto status_item = new QTableWidgetItem("🟢 Active");
                status_item->setBackground(QColor(200, 255, 200));
                robots_table_->setItem(row_idx, 2, status_item);
                
                // Last seen (Current time)
                auto time_item = new QTableWidgetItem(QTime::currentTime().toString("HH:mm:ss"));
                robots_table_->setItem(row_idx, 3, time_item);
                
                row_idx++;
            }
        }
    }
}

void NetworkTab::on_discover_robots_clicked() {
    refresh_network_status();
}

void NetworkTab::on_refresh_status_clicked() {
    refresh_network_status();
}

void NetworkTab::create_ui_() {
    auto layout = new QVBoxLayout();

    auto buttons_layout = new QHBoxLayout();
    discover_button_ = new QPushButton("Discover Robots");
    refresh_button_ = new QPushButton("Refresh");
    connect(discover_button_, &QPushButton::clicked,
            this, &NetworkTab::on_discover_robots_clicked);
    connect(refresh_button_, &QPushButton::clicked,
            this, &NetworkTab::on_refresh_status_clicked);
    buttons_layout->addWidget(discover_button_);
    buttons_layout->addWidget(refresh_button_);
    buttons_layout->addStretch();

    network_status_label_ = new QLabel("Status: Unknown");

    robots_table_ = new QTableWidget();
    robots_table_->setColumnCount(4);
    robots_table_->setHorizontalHeaderLabels(
        {"Robot Name", "IP Address", "Status", "Last Seen"});

    layout->addLayout(buttons_layout);
    layout->addWidget(network_status_label_);
    layout->addWidget(robots_table_);
    setLayout(layout);
}

}  // namespace ros2_dashboard::gui
