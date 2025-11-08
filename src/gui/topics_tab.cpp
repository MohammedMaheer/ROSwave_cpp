/**
 * @file gui/topics_tab.cpp
 */

#include "gui/topics_tab.hpp"
#include "ros2_manager.hpp"
#include "async_worker.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QSplitter>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QTextEdit>
#include <iostream>

namespace ros2_dashboard::gui {

TopicsTab::TopicsTab(QWidget* parent)
    : QWidget(parent) {
    create_ui_();
    
    // Connect signals for thread-safe UI updates
    connect(this, &TopicsTab::on_topics_updated,
            this, &TopicsTab::on_topics_updated_slot,
            Qt::QueuedConnection);
    
    connect(this, &TopicsTab::on_message_updated,
            this, &TopicsTab::on_message_updated_slot,
            Qt::QueuedConnection);
    
    // NOTE: Auto-refresh timer is now managed by MainWindow's timer coordinator
    // This ensures only the active tab queries ROS2, reducing CPU and network usage
    // Timer was: refresh_timer_ = new QTimer(this); refresh_timer_->start(3000);
}

TopicsTab::~TopicsTab() = default;

void TopicsTab::initialize(std::shared_ptr<ROS2Manager> ros2_manager,
                          std::shared_ptr<AsyncWorker> async_worker) {
    ros2_manager_ = ros2_manager;
    async_worker_ = async_worker;
    refresh_topics();
}

void TopicsTab::refresh_topics() {
    if (ros2_manager_) {
        std::cerr << "[TopicsTab] Queuing refresh_topics task..." << std::endl;
        async_worker_->enqueue([this]() {
            std::cerr << "[TopicsTab] Executing refresh_topics in worker thread..." << std::endl;
            auto topics = ros2_manager_->get_topics();
            std::cerr << "[TopicsTab] Got " << topics.size() << " topics" << std::endl;
            // Emit signal to update UI from main thread
            emit on_topics_updated(topics);
        });
    } else {
        std::cerr << "[TopicsTab] ros2_manager_ is null!" << std::endl;
    }
}

void TopicsTab::on_topic_selected(int row, int column) {
    if (row < 0 || row >= topics_table_->rowCount()) return;
    
    QString topic_name = topics_table_->item(row, 0)->text();
    // Remove emoji if present
    if (topic_name.startsWith("🔵")) {
        topic_name = topic_name.mid(2);
    }
    
    if (ros2_manager_) {
        async_worker_->enqueue([this, topic_name]() {
            auto msg = ros2_manager_->get_topic_echo(topic_name.toStdString());
            if (msg.has_value()) {
                // Emit signal to update UI from main thread
                emit on_message_updated(QString::fromStdString(msg.value()));
            } else {
                emit on_message_updated("(No message available)");
            }
        });
    }
}

void TopicsTab::on_refresh_button_clicked() {
    refresh_topics();
}

void TopicsTab::on_topics_updated_slot(const std::vector<ros2_dashboard::TopicInfo>& topics) {
    // This is called from the main Qt thread - safe to update UI
    update_topics_table(topics);
}

void TopicsTab::on_message_updated_slot(const QString& message) {
    // This is called from the main Qt thread - safe to update UI
    message_preview_->setText(message);
}

void TopicsTab::create_ui_() {
    auto main_layout = new QVBoxLayout();
    
    // Top control bar
    auto top_layout = new QHBoxLayout();
    auto refresh_btn = new QPushButton("🔄 Refresh", this);
    connect(refresh_btn, &QPushButton::clicked, this, &TopicsTab::on_refresh_button_clicked);
    
    auto total_label = new QLabel("Total Topics: ", this);
    total_topics_label_ = new QLabel("0", this);
    total_topics_label_->setStyleSheet("font-weight: bold; color: blue;");
    
    top_layout->addWidget(refresh_btn);
    top_layout->addWidget(total_label);
    top_layout->addWidget(total_topics_label_);
    top_layout->addStretch();
    
    main_layout->addLayout(top_layout);
    
    // Create splitter for topics table and message preview
    auto splitter = new QSplitter(Qt::Vertical, this);
    
    // Topics table
    topics_table_ = new QTableWidget();
    topics_table_->setColumnCount(4);
    topics_table_->setHorizontalHeaderLabels({
        "🔵 Topic Name",
        "📦 Type",
        "📤 Pubs",
        "📥 Subs"
    });
    
    topics_table_->horizontalHeader()->setStretchLastSection(false);
    topics_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    topics_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    topics_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    topics_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
    topics_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    topics_table_->setAlternatingRowColors(true);
    
    connect(topics_table_, &QTableWidget::cellClicked, 
            this, &TopicsTab::on_topic_selected);
    
    splitter->addWidget(topics_table_);
    
    // Message preview
    auto preview_label = new QLabel("Message Preview:", this);
    preview_label->setStyleSheet("font-weight: bold;");
    message_preview_ = new QTextEdit(this);
    message_preview_->setReadOnly(true);
    message_preview_->setMaximumHeight(150);
    message_preview_->setPlaceholderText("Select a topic to see its latest message...");
    
    auto preview_container = new QWidget();
    auto preview_layout = new QVBoxLayout();
    preview_layout->addWidget(preview_label);
    preview_layout->addWidget(message_preview_);
    preview_container->setLayout(preview_layout);
    
    splitter->addWidget(preview_container);
    splitter->setStretchFactor(0, 2);
    splitter->setStretchFactor(1, 1);
    
    main_layout->addWidget(splitter);
    
    setLayout(main_layout);
}

void TopicsTab::update_topics_table(const std::vector<ros2_dashboard::TopicInfo>& topics) {
    topics_table_->setRowCount(0);
    
    for (const auto& topic : topics) {
        int row = topics_table_->rowCount();
        topics_table_->insertRow(row);
        
        // Topic name with emoji
        auto name_item = new QTableWidgetItem("🔵 " + QString::fromStdString(topic.name));
        name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
        topics_table_->setItem(row, 0, name_item);
        
        // Type
        auto type_item = new QTableWidgetItem(QString::fromStdString(topic.msg_type));
        type_item->setFlags(type_item->flags() & ~Qt::ItemIsEditable);
        topics_table_->setItem(row, 1, type_item);
        
        // Publishers count
        auto pubs_item = new QTableWidgetItem(QString::number(topic.publisher_count));
        pubs_item->setFlags(pubs_item->flags() & ~Qt::ItemIsEditable);
        pubs_item->setTextAlignment(Qt::AlignCenter);
        topics_table_->setItem(row, 2, pubs_item);
        
        // Subscribers count
        auto subs_item = new QTableWidgetItem(QString::number(topic.subscription_count));
        subs_item->setFlags(subs_item->flags() & ~Qt::ItemIsEditable);
        subs_item->setTextAlignment(Qt::AlignCenter);
        topics_table_->setItem(row, 3, subs_item);
    }
    
    total_topics_label_->setText(QString::number(topics.size()));
}

}  // namespace ros2_dashboard::gui
