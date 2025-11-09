/**
 * @file gui/services_tab.cpp
 */

#include "gui/services_tab.hpp"
#include "ros2_manager.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QHeaderView>

namespace ros2_dashboard::gui {

ServicesTab::ServicesTab(QWidget* parent) : QWidget(parent) {
    create_ui_();
    
    // Connect signal for thread-safe UI updates
    connect(this, &ServicesTab::on_services_updated,
            this, &ServicesTab::on_services_updated_slot,
            Qt::QueuedConnection);
    
    // NOTE: Auto-refresh timer is now managed by MainWindow's timer coordinator
    // This ensures only the active tab queries ROS2, reducing CPU and network usage
    // Timer was: refresh_timer_ = new QTimer(this); refresh_timer_->start(2000);
}

ServicesTab::~ServicesTab() = default;

void ServicesTab::initialize(std::shared_ptr<ROS2Manager> ros2_manager,
                            std::shared_ptr<AsyncWorker> async_worker) {
    ros2_manager_ = ros2_manager;
    async_worker_ = async_worker;
    refresh_services();
}

void ServicesTab::refresh_services() {
    if (ros2_manager_) {
        async_worker_->enqueue([this]() {
            auto services = ros2_manager_->get_services();
            emit on_services_updated(services);
        });
    }
}

void ServicesTab::on_refresh_button_clicked() {
    refresh_services();
}

void ServicesTab::on_services_updated_slot(const std::vector<ros2_dashboard::ServiceInfo>& services) {
    update_services_table(services);
}

void ServicesTab::create_ui_() {
    auto layout = new QVBoxLayout();
    
    // Top control bar
    auto top_layout = new QHBoxLayout();
    auto refresh_btn = new QPushButton("🔄 Refresh", this);
    connect(refresh_btn, &QPushButton::clicked, this, &ServicesTab::on_refresh_button_clicked);
    
    auto total_label = new QLabel("Total Services: ", this);
    total_services_label_ = new QLabel("0", this);
    total_services_label_->setStyleSheet("font-weight: bold; color: purple;");
    
    top_layout->addWidget(refresh_btn);
    top_layout->addWidget(total_label);
    top_layout->addWidget(total_services_label_);
    top_layout->addStretch();
    
    layout->addLayout(top_layout);
    
    // Services table
    services_table_ = new QTableWidget();
    services_table_->setColumnCount(3);
    services_table_->setHorizontalHeaderLabels({
        "📡 Service Name",
        "📝 Type",
        "🖥️ Servers"
    });
    services_table_->horizontalHeader()->setStretchLastSection(false);
    services_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    services_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    services_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    services_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    services_table_->setAlternatingRowColors(true);
    // Enable scrollbars
    services_table_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    services_table_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    
    layout->addWidget(services_table_);
    setLayout(layout);
}

void ServicesTab::update_services_table(const std::vector<ros2_dashboard::ServiceInfo>& services) {
    services_table_->setRowCount(0);
    
    for (const auto& service : services) {
        int row = services_table_->rowCount();
        services_table_->insertRow(row);
        
        // Service name with emoji
        auto name_item = new QTableWidgetItem("📡 " + QString::fromStdString(service.name));
        name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
        services_table_->setItem(row, 0, name_item);
        
        // Type
        auto type_item = new QTableWidgetItem(QString::fromStdString(service.service_type));
        type_item->setFlags(type_item->flags() & ~Qt::ItemIsEditable);
        services_table_->setItem(row, 1, type_item);
        
        // Server count
        auto servers_item = new QTableWidgetItem(QString::number(service.servers.size()));
        servers_item->setFlags(servers_item->flags() & ~Qt::ItemIsEditable);
        servers_item->setTextAlignment(Qt::AlignCenter);
        services_table_->setItem(row, 2, servers_item);
    }
    
    total_services_label_->setText(QString::number(services.size()));
}

}  // namespace ros2_dashboard::gui
