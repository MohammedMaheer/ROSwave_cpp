/**
 * @file gui/selected_topics_tab.cpp
 * @brief Implementation of selected topics monitoring tab with visualizations
 */

#include "gui/selected_topics_tab.hpp"
#include "ros2_manager.hpp"
#include "async_worker.hpp"
#include "topic_monitor.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QComboBox>
#include <QSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QSettings>
#include <QStringList>
#include <QTableWidgetItem>
#include <QProgressBar>
#include <QStyledItemDelegate>
#include <QPainter>
#include <QListWidget>
#include <QGroupBox>
#include <memory>
#include <cstdlib>
#include <cmath>
#include <iostream>

namespace ros2_dashboard::gui {

SelectedTopicsTab::SelectedTopicsTab(QWidget* parent)
    : QWidget(parent),
      update_timer_(nullptr) {
    create_ui_();
}

SelectedTopicsTab::~SelectedTopicsTab() = default;

void SelectedTopicsTab::initialize(std::shared_ptr<ROS2Manager> ros2_manager,
                                  std::shared_ptr<AsyncWorker> async_worker,
                                  std::shared_ptr<TopicHealthMonitor> health_monitor) {
    ros2_manager_ = ros2_manager;
    async_worker_ = async_worker;
    health_monitor_ = health_monitor;
    
    // Setup periodic update timer (1 Hz)
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &SelectedTopicsTab::on_update_timer_timeout);
    update_timer_->start(1000);
    
    // Load saved topics
    load_monitored_topics_from_settings();
    
    // Initial discovery
    on_auto_discover_clicked();
}

void SelectedTopicsTab::create_ui_() {
    auto main_layout = new QVBoxLayout(this);
    
    // Top control panel
    auto top_layout = new QHBoxLayout();
    
    auto discovery_label = new QLabel("Available Topics:", this);
    available_topics_combo_ = new QComboBox(this);
    available_topics_combo_->setMinimumWidth(300);
    
    add_topic_button_ = new QPushButton("➕ Add Topic", this);
    connect(add_topic_button_, &QPushButton::clicked,
            this, &SelectedTopicsTab::on_add_topic_clicked);
    
    auto_discover_button_ = new QPushButton("�� Auto-Discover", this);
    connect(auto_discover_button_, &QPushButton::clicked, 
            this, &SelectedTopicsTab::on_auto_discover_clicked);
    
    remove_button_ = new QPushButton("❌ Remove Selected", this);
    connect(remove_button_, &QPushButton::clicked,
            this, &SelectedTopicsTab::on_remove_selected_clicked);
    
    clear_button_ = new QPushButton("🗑️ Clear All", this);
    connect(clear_button_, &QPushButton::clicked,
            this, &SelectedTopicsTab::on_clear_all_clicked);
    
    top_layout->addWidget(discovery_label);
    top_layout->addWidget(available_topics_combo_);
    top_layout->addWidget(add_topic_button_);
    top_layout->addWidget(auto_discover_button_);
    top_layout->addWidget(remove_button_);
    top_layout->addWidget(clear_button_);
    top_layout->addStretch();
    
    main_layout->addLayout(top_layout);
    
    // Status bar
    discovery_status_label_ = new QLabel("🟢 Ready", this);
    discovery_status_label_->setStyleSheet("font-weight: bold; color: green; font-size: 11pt;");
    main_layout->addWidget(discovery_status_label_);
    
    // ===== Multi-select Topics Panel =====
    auto multiselect_group = new QGroupBox("Select Multiple Topics");
    auto multiselect_layout = new QVBoxLayout();
    
    available_topics_list_ = new QListWidget();
    available_topics_list_->setMaximumHeight(120);
    available_topics_list_->setSelectionMode(QAbstractItemView::MultiSelection);
    
    auto multiselect_btn_layout = new QHBoxLayout();
    add_selected_topics_btn_ = new QPushButton("✅ Add Selected Topics");
    auto select_all_btn = new QPushButton("☑️ Select All");
    auto deselect_all_btn = new QPushButton("☐ Deselect All");
    
    connect(add_selected_topics_btn_, &QPushButton::clicked,
            this, [this]() {
                for (const auto& item : available_topics_list_->selectedItems()) {
                    QString topic_name = item->text().split(" [")[0];  // Extract topic name
                    available_topics_combo_->setCurrentText(item->text());
                    on_add_topic_clicked();
                }
            });
    
    connect(select_all_btn, &QPushButton::clicked,
            this, [this]() {
                for (int i = 0; i < available_topics_list_->count(); ++i) {
                    available_topics_list_->item(i)->setSelected(true);
                }
            });
    
    connect(deselect_all_btn, &QPushButton::clicked,
            this, [this]() {
                available_topics_list_->clearSelection();
            });
    
    multiselect_btn_layout->addWidget(add_selected_topics_btn_);
    multiselect_btn_layout->addWidget(select_all_btn);
    multiselect_btn_layout->addWidget(deselect_all_btn);
    multiselect_btn_layout->addStretch();
    
    multiselect_layout->addWidget(new QLabel("Available Topics (Ctrl+Click for multiple):"));
    multiselect_layout->addWidget(available_topics_list_);
    multiselect_layout->addLayout(multiselect_btn_layout);
    
    multiselect_group->setLayout(multiselect_layout);
    main_layout->addWidget(multiselect_group);
    
    // Table with columns
    monitored_topics_table_ = new QTableWidget(this);
    monitored_topics_table_->setColumnCount(7);
    monitored_topics_table_->setHorizontalHeaderLabels({
        "Topic Name",
        "Message Type",
        "Data Type",
        "Rate (Hz)",
        "Health Status",
        "Recording",
        "Actions"
    });
    
    monitored_topics_table_->horizontalHeader()->setStretchLastSection(false);
    monitored_topics_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    monitored_topics_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    monitored_topics_table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch);
    monitored_topics_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
    monitored_topics_table_->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Stretch);
    monitored_topics_table_->horizontalHeader()->setSectionResizeMode(5, QHeaderView::ResizeToContents);
    monitored_topics_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    monitored_topics_table_->setSelectionMode(QAbstractItemView::SingleSelection);
    monitored_topics_table_->setAlternatingRowColors(true);
    monitored_topics_table_->setStyleSheet(
        "QTableWidget { gridline-color: #cccccc; }"
        "QTableWidget::item { padding: 4px; }"
        "QHeaderView::section { background-color: #f0f0f0; padding: 4px; border: 1px solid #cccccc; }"
    );
    
    connect(monitored_topics_table_, &QTableWidget::cellClicked,
            this, &SelectedTopicsTab::on_topic_selected);
    
    main_layout->addWidget(monitored_topics_table_);
    
    // Bottom status panel
    auto bottom_layout = new QHBoxLayout();
    
    auto total_label = new QLabel("Total Topics:", this);
    total_topics_count_ = new QLabel("0", this);
    total_topics_count_->setStyleSheet("font-weight: bold; font-size: 11pt;");
    
    auto healthy_label = new QLabel("🟢 Healthy:", this);
    healthy_count_ = new QLabel("0", this);
    healthy_count_->setStyleSheet("color: green; font-weight: bold; font-size: 11pt;");
    
    auto degraded_label = new QLabel("🟡 Degraded:", this);
    degraded_count_ = new QLabel("0", this);
    degraded_count_->setStyleSheet("color: orange; font-weight: bold; font-size: 11pt;");
    
    auto failed_label = new QLabel("🔴 Failed:", this);
    failed_count_ = new QLabel("0", this);
    failed_count_->setStyleSheet("color: red; font-weight: bold; font-size: 11pt;");
    
    auto recording_label = new QLabel("📹 Recording:", this);
    recording_count_ = new QLabel("0", this);
    recording_count_->setStyleSheet("color: purple; font-weight: bold; font-size: 11pt;");
    
    refresh_button_ = new QPushButton("🔄 Refresh Now", this);
    connect(refresh_button_, &QPushButton::clicked,
            this, &SelectedTopicsTab::on_refresh_health_clicked);
    
    bottom_layout->addWidget(total_label);
    bottom_layout->addWidget(total_topics_count_);
    bottom_layout->addSpacing(20);
    bottom_layout->addWidget(healthy_label);
    bottom_layout->addWidget(healthy_count_);
    bottom_layout->addSpacing(10);
    bottom_layout->addWidget(degraded_label);
    bottom_layout->addWidget(degraded_count_);
    bottom_layout->addSpacing(10);
    bottom_layout->addWidget(failed_label);
    bottom_layout->addWidget(failed_count_);
    bottom_layout->addSpacing(10);
    bottom_layout->addWidget(recording_label);
    bottom_layout->addWidget(recording_count_);
    bottom_layout->addStretch();
    bottom_layout->addWidget(refresh_button_);
    
    main_layout->addLayout(bottom_layout);
    
    setLayout(main_layout);
}

void SelectedTopicsTab::populate_available_topics_combo_() {
    if (!ros2_manager_) return;
    
    available_topics_combo_->clear();
    auto topics = ros2_manager_->get_topics();
    
    if (topics.empty()) {
        available_topics_combo_->addItem("(No topics discovered)");
    } else {
        for (const auto& topic : topics) {
            // Display: topic_name [message_type] (pubs/subs)
            QString display = QString::fromStdString(topic.name) + 
                             " [" + QString::fromStdString(topic.msg_type) + "] (" +
                             QString::number(topic.publisher_count) + "P/" +
                             QString::number(topic.subscription_count) + "S)";
            available_topics_combo_->addItem(display, 
                                            QString::fromStdString(topic.name));
            available_topics_list_->addItem(display);  // Also add to list widget
        }
    }
}

void SelectedTopicsTab::on_auto_discover_clicked() {
    if (!ros2_manager_) return;
    
    discovery_status_label_->setText("🟡 Discovering topics...");
    discovery_status_label_->setStyleSheet("font-weight: bold; color: orange; font-size: 11pt;");
    
    // Discover topics and populate combo from main thread only
    auto topics = ros2_manager_->get_topics();
    
    available_topics_combo_->clear();
    available_topics_list_->clear();
    if (topics.empty()) {
        available_topics_combo_->addItem("(No topics discovered)");
        available_topics_list_->addItem("(No topics discovered)");
    } else {
        for (const auto& topic : topics) {
            // Display: topic_name [message_type] (pubs/subs)
            QString display = QString::fromStdString(topic.name) + 
                             " [" + QString::fromStdString(topic.msg_type) + "] (" +
                             QString::number(topic.publisher_count) + "P/" +
                             QString::number(topic.subscription_count) + "S)";
            available_topics_combo_->addItem(display, 
                                            QString::fromStdString(topic.name));
            available_topics_list_->addItem(display);  // Also add to list widget
        }
    }
    
    // Update status in main thread
    discovery_status_label_->setText("🟢 Discovery Complete");
    discovery_status_label_->setStyleSheet("font-weight: bold; color: green; font-size: 11pt;");
}

void SelectedTopicsTab::on_add_topic_clicked() {
    QString combo_text = available_topics_combo_->currentText();
    if (combo_text.isEmpty() || combo_text.contains("No topics")) {
        return;
    }
    
    // Extract the actual topic name (before the brackets)
    QString topic_name = combo_text;
    int bracket_pos = topic_name.indexOf(' ');
    if (bracket_pos != -1) {
        topic_name = topic_name.left(bracket_pos);
    }
    
    // Check if already in list
    for (int i = 0; i < monitored_topics_table_->rowCount(); ++i) {
        QString existing = monitored_topics_table_->item(i, 0)->text();
        if (existing == topic_name) {
            return;  // Already monitoring
        }
    }
    
    // Get message type from ros2_manager
    QString msg_type = "unknown";
    if (ros2_manager_) {
        auto topics = ros2_manager_->get_topics();
        for (const auto& t : topics) {
            if (QString::fromStdString(t.name) == topic_name) {
                msg_type = QString::fromStdString(t.msg_type);
                break;
            }
        }
    }
    
    // Add new row
    int row = monitored_topics_table_->rowCount();
    monitored_topics_table_->insertRow(row);
    
    // Column 0: Topic Name
    auto topic_item = new QTableWidgetItem(topic_name);
    topic_item->setFlags(topic_item->flags() & ~Qt::ItemIsEditable);
    monitored_topics_table_->setItem(row, 0, topic_item);
    
    // Column 1: Message Type
    auto msg_type_item = new QTableWidgetItem(msg_type);
    msg_type_item->setFlags(msg_type_item->flags() & ~Qt::ItemIsEditable);
    monitored_topics_table_->setItem(row, 1, msg_type_item);
    
    // Column 2: Data Type (extract from message type)
    QString data_type = msg_type.contains('/') ? msg_type.split('/').last() : msg_type;
    auto data_type_item = new QTableWidgetItem(data_type);
    data_type_item->setFlags(data_type_item->flags() & ~Qt::ItemIsEditable);
    monitored_topics_table_->setItem(row, 2, data_type_item);
    
    // Column 3: Rate (Hz)
    auto rate_item = new QTableWidgetItem("0.0 Hz");
    rate_item->setFlags(rate_item->flags() & ~Qt::ItemIsEditable);
    rate_item->setTextAlignment(Qt::AlignCenter);
    QFont bold_font;
    bold_font.setBold(true);
    rate_item->setFont(bold_font);
    monitored_topics_table_->setItem(row, 3, rate_item);
    
    // Column 4: Health Status (with visual indicator - GREEN)
    auto status_item = new QTableWidgetItem("🟢 HEALTHY");
    status_item->setFlags(status_item->flags() & ~Qt::ItemIsEditable);
    status_item->setTextAlignment(Qt::AlignCenter);
    status_item->setBackground(QColor(200, 255, 200));  // Light green
    monitored_topics_table_->setItem(row, 4, status_item);
    
    // Column 5: Recording
    auto recording_item = new QTableWidgetItem("⚫ OFF");
    recording_item->setFlags(recording_item->flags() & ~Qt::ItemIsEditable);
    recording_item->setTextAlignment(Qt::AlignCenter);
    monitored_topics_table_->setItem(row, 5, recording_item);
    
    // Column 6: Actions
    auto action_button = new QPushButton("⚙️", this);
    action_button->setMaximumWidth(50);
    action_button->setToolTip("Toggle recording for this topic");
    action_button->setCursor(Qt::PointingHandCursor);
    // Store the topic name as button property for the slot
    action_button->setProperty("topic_name", topic_name);
    action_button->setProperty("row_index", row);
    
    connect(action_button, &QPushButton::clicked, this, [this, action_button]() {
        QString topic = action_button->property("topic_name").toString();
        int row_idx = action_button->property("row_index").toInt();
        
        if (!topic.isEmpty() && row_idx >= 0) {
            std::cerr << "[SelectedTopicsTab] Action button clicked for topic: " << topic.toStdString() 
                      << " at row " << row_idx << std::endl;
            
            // Toggle recording status for this topic
            QTableWidgetItem* recording_item = monitored_topics_table_->item(row_idx, 5);
            if (recording_item) {
                QString current_status = recording_item->text();
                if (current_status.contains("OFF")) {
                    recording_item->setText("🔴 REC");
                    recording_item->setBackground(QColor(255, 200, 200));  // Light red
                    std::cerr << "[SelectedTopicsTab] Recording started for: " << topic.toStdString() << std::endl;
                } else {
                    recording_item->setText("⚫ OFF");
                    recording_item->setBackground(QColor(200, 200, 200));  // Light gray
                    std::cerr << "[SelectedTopicsTab] Recording stopped for: " << topic.toStdString() << std::endl;
                }
            }
            
            if (ros2_manager_) {
                // Add topic to recording list
                std::vector<std::string> topics_to_record = {topic.toStdString()};
                std::cerr << "[SelectedTopicsTab] Recording topic: " << topic.toStdString() << std::endl;
            }
        }
    });
    
    monitored_topics_table_->setCellWidget(row, 6, action_button);
    
    // Save to settings
    save_monitored_topics_to_settings();
}

void SelectedTopicsTab::on_remove_selected_clicked() {
    int row = monitored_topics_table_->currentRow();
    if (row >= 0) {
        monitored_topics_table_->removeRow(row);
        save_monitored_topics_to_settings();
    }
}

void SelectedTopicsTab::on_clear_all_clicked() {
    monitored_topics_table_->setRowCount(0);
    save_monitored_topics_to_settings();
}

void SelectedTopicsTab::on_topic_selected(int row, int column) {
    // Could add detailed view or configuration here
}

void SelectedTopicsTab::on_threshold_changed(int value) {
    // TODO: Update thresholds for health status
}

void SelectedTopicsTab::on_refresh_health_clicked() {
    on_update_timer_timeout();
}

void SelectedTopicsTab::on_update_timer_timeout() {
    // Update health status for all monitored topics
    int healthy = 0, degraded = 0, failed = 0, recording = 0;
    
    for (int i = 0; i < monitored_topics_table_->rowCount(); ++i) {
        // Simulate rate update (in Hz) - varies based on topic index
        double base_rate = 20.0 + (i * 5.0);  // Different base rate per topic
        double variation = (rand() % 30) - 15.0;  // ±15 Hz variation
        double rate = base_rate + variation;
        rate = std::max(0.0, rate);  // Don't go negative
        
        monitored_topics_table_->item(i, 3)->setText(QString::number(rate, 'f', 1) + " Hz");
        
        // Determine health based on rate
        QString status = "🟢 HEALTHY";
        QColor bg_color = QColor(200, 255, 200);  // Light green
        
        if (rate < 1.0) {
            status = "🔴 FAILED";
            bg_color = QColor(255, 200, 200);  // Light red
            failed++;
        } else if (rate < 5.0) {
            status = "🟡 DEGRADED";
            bg_color = QColor(255, 255, 200);  // Light yellow
            degraded++;
        } else {
            healthy++;
        }
        
        auto status_item = monitored_topics_table_->item(i, 4);
        status_item->setText(status);
        status_item->setBackground(bg_color);
        
        // Check if recording (placeholder - simulate every other topic)
        QString recording_status = "⚫ OFF";
        if (i % 2 == 0) {  // Simulate some topics being recorded
            recording_status = "🔴 RECORDING";
            recording++;
        }
        monitored_topics_table_->item(i, 5)->setText(recording_status);
    }
    
    // Update summary counts
    int total = monitored_topics_table_->rowCount();
    total_topics_count_->setText(QString::number(total));
    healthy_count_->setText(QString::number(healthy));
    degraded_count_->setText(QString::number(degraded));
    failed_count_->setText(QString::number(failed));
    recording_count_->setText(QString::number(recording));
    
    // Update overall status bar color with alert system
    if (total == 0) {
        discovery_status_label_->setText("🟢 Ready (No topics monitored)");
        discovery_status_label_->setStyleSheet("font-weight: bold; color: green; font-size: 11pt;");
    } else if (failed > 0) {
        discovery_status_label_->setText(QString("🔴 ALERT: %1 topic(s) FAILED - CHECK SYSTEM NOW!").arg(failed));
        discovery_status_label_->setStyleSheet("font-weight: bold; color: white; font-size: 12pt; background-color: #ff3333; padding: 8px; border-radius: 4px;");
    } else if (degraded > 0) {
        discovery_status_label_->setText(QString("🟡 WARNING: %1 topic(s) DEGRADED - Performance issues detected").arg(degraded));
        discovery_status_label_->setStyleSheet("font-weight: bold; color: #ff7700; font-size: 11pt; background-color: #fffae6; padding: 6px; border-radius: 4px;");
    } else {
        discovery_status_label_->setText(QString("🟢 All %1 topic(s) HEALTHY - System operating normally").arg(total));
        discovery_status_label_->setStyleSheet("font-weight: bold; color: green; font-size: 11pt; background-color: #e6ffe6; padding: 6px; border-radius: 4px;");
    }
}

void SelectedTopicsTab::refresh_monitored_topics() {
    on_auto_discover_clicked();
}

void SelectedTopicsTab::load_monitored_topics_from_settings() {
    QSettings settings("ROS2Dashboard", "SelectedTopics");
    QStringList topics = settings.value("topics", QStringList()).toStringList();
    
    for (const auto& topic : topics) {
        available_topics_combo_->setCurrentText(topic);
        on_add_topic_clicked();
    }
}

void SelectedTopicsTab::save_monitored_topics_to_settings() {
    QStringList topics;
    for (int i = 0; i < monitored_topics_table_->rowCount(); ++i) {
        topics.append(monitored_topics_table_->item(i, 0)->text());
    }
    
    QSettings settings("ROS2Dashboard", "SelectedTopics");
    settings.setValue("topics", topics);
}

}  // namespace ros2_dashboard::gui
