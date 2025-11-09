/**
 * @file gui/nodes_tab.cpp
 */

#include "gui/nodes_tab.hpp"
#include "ros2_manager.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QTreeWidgetItem>

namespace ros2_dashboard::gui {

NodesTab::NodesTab(QWidget* parent) : QWidget(parent) {
    create_ui_();
    
    // Connect signal for thread-safe UI updates
    connect(this, &NodesTab::on_nodes_updated,
            this, &NodesTab::on_nodes_updated_slot,
            Qt::QueuedConnection);
    
    // NOTE: Auto-refresh timer is now managed by MainWindow's timer coordinator
    // This ensures only the active tab queries ROS2, reducing CPU and network usage
    // Timer was: refresh_timer_ = new QTimer(this); refresh_timer_->start(2000);
}

NodesTab::~NodesTab() = default;

void NodesTab::initialize(std::shared_ptr<ROS2Manager> ros2_manager,
                         std::shared_ptr<AsyncWorker> async_worker) {
    ros2_manager_ = ros2_manager;
    async_worker_ = async_worker;
    refresh_nodes();
}

void NodesTab::refresh_nodes() {
    if (ros2_manager_) {
        async_worker_->enqueue([this]() {
            auto nodes = ros2_manager_->get_nodes();
            // Emit signal to update UI from main thread
            emit on_nodes_updated(nodes);
        });
    }
}

void NodesTab::on_refresh_button_clicked() {
    refresh_nodes();
}

void NodesTab::on_nodes_updated_slot(const std::vector<ros2_dashboard::NodeInfo>& nodes) {
    // This is called from the main Qt thread - safe to update UI
    update_nodes_tree(nodes);
}

void NodesTab::create_ui_() {
    auto layout = new QVBoxLayout();
    
    // Top control bar
    auto top_layout = new QHBoxLayout();
    auto refresh_btn = new QPushButton("🔄 Refresh", this);
    connect(refresh_btn, &QPushButton::clicked, this, &NodesTab::on_refresh_button_clicked);
    
    auto total_label = new QLabel("Total Nodes: ", this);
    total_nodes_label_ = new QLabel("0", this);
    total_nodes_label_->setStyleSheet("font-weight: bold; color: green;");
    
    top_layout->addWidget(refresh_btn);
    top_layout->addWidget(total_label);
    top_layout->addWidget(total_nodes_label_);
    top_layout->addStretch();
    
    layout->addLayout(top_layout);
    
    // Nodes tree
    nodes_tree_ = new QTreeWidget();
    nodes_tree_->setColumnCount(3);
    nodes_tree_->setHeaderLabels({"🔗 Node/Info", "Type", "Status"});
    nodes_tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    nodes_tree_->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    nodes_tree_->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    nodes_tree_->setAlternatingRowColors(true);
    // Enable scrollbars
    nodes_tree_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    nodes_tree_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    
    layout->addWidget(nodes_tree_);
    
    setLayout(layout);
}

void NodesTab::update_nodes_tree(const std::vector<ros2_dashboard::NodeInfo>& nodes) {
    nodes_tree_->clear();
    
    for (const auto& node : nodes) {
        auto node_item = new QTreeWidgetItem();
        node_item->setText(0, "🟢 " + QString::fromStdString(node.name));
        node_item->setText(1, "Node");
        node_item->setText(2, "Active");
        node_item->setFlags(node_item->flags() & ~Qt::ItemIsEditable);
        
        // Add sub-items for publishers
        if (!node.publications.empty()) {
            auto pub_item = new QTreeWidgetItem(node_item);
            pub_item->setText(0, "📤 Publishers (" + QString::number(node.publications.size()) + ")");
            pub_item->setText(1, "Topics");
            pub_item->setFlags(pub_item->flags() & ~Qt::ItemIsEditable);
            
            for (const auto& topic : node.publications) {
                auto topic_item = new QTreeWidgetItem(pub_item);
                topic_item->setText(0, "  ▸ " + QString::fromStdString(topic));
                topic_item->setFlags(topic_item->flags() & ~Qt::ItemIsEditable);
            }
        }
        
        // Add sub-items for subscribers
        if (!node.subscriptions.empty()) {
            auto sub_item = new QTreeWidgetItem(node_item);
            sub_item->setText(0, "📥 Subscribers (" + QString::number(node.subscriptions.size()) + ")");
            sub_item->setText(1, "Topics");
            sub_item->setFlags(sub_item->flags() & ~Qt::ItemIsEditable);
            
            for (const auto& topic : node.subscriptions) {
                auto topic_item = new QTreeWidgetItem(sub_item);
                topic_item->setText(0, "  ▸ " + QString::fromStdString(topic));
                topic_item->setFlags(topic_item->flags() & ~Qt::ItemIsEditable);
            }
        }
        
        nodes_tree_->addTopLevelItem(node_item);
    }
    
    total_nodes_label_->setText(QString::number(nodes.size()));
}

}  // namespace ros2_dashboard::gui
