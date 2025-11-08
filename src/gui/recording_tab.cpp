/**
 * @file gui/recording_tab.cpp
 */

#include "gui/recording_tab.hpp"
#include "ros2_manager.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFileDialog>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QDir>
#include <QComboBox>
#include <QLineEdit>
#include <QTableWidget>
#include <QTimer>
#include <QStorageInfo>
#include <QFileInfo>
#include <iostream>
#include <algorithm>
#include <cstdio>

namespace ros2_dashboard::gui {

RecordingTab::RecordingTab(QWidget* parent) : QWidget(parent) { 
    create_ui_();
    
    // Setup auto-refresh timer for recording status
    status_refresh_timer_ = new QTimer(this);
    connect(status_refresh_timer_, &QTimer::timeout, this, &RecordingTab::refresh_recording_status);
    status_refresh_timer_->start(1000);  // Refresh every 1 second
    
    // Setup timer for bag list refresh
    bag_list_refresh_timer_ = new QTimer(this);
    connect(bag_list_refresh_timer_, &QTimer::timeout, this, &RecordingTab::refresh_bag_list);
    bag_list_refresh_timer_->start(2000);  // Refresh every 2 seconds
}
RecordingTab::~RecordingTab() = default;

void RecordingTab::initialize(std::shared_ptr<ROS2Manager> ros2_manager,
                             std::shared_ptr<AsyncWorker> async_worker) {
    ros2_manager_ = ros2_manager;
    async_worker_ = async_worker;
}

void RecordingTab::refresh_recording_status() {
    if (ros2_manager_) {
        // Call directly in main thread (do NOT use async_worker for UI updates)
        // Qt widgets MUST be updated from the main thread only
        update_recording_controls_();
    }
}

void RecordingTab::refresh_bag_list() {
    if (ros2_manager_) {
        // Call directly in main thread (do NOT use async_worker for UI updates)
        // Qt widgets MUST be updated from the main thread only
        update_bag_list_();
    }
}

void RecordingTab::on_start_recording_clicked() {
    QString output_dir = output_dir_edit_->text();
    
    // Validate output directory
    if (output_dir.isEmpty()) {
        status_label_->setText("Status: Error - No output directory selected");
        return;
    }
    
    // Create directory if it doesn't exist
    QDir dir(output_dir);
    if (!dir.exists()) {
        if (!dir.mkpath(".")) {
            status_label_->setText("Status: Error - Could not create output directory");
            return;
        }
    }
    
    // Check write permissions
    QFileInfo fileInfo(output_dir);
    if (!fileInfo.isWritable()) {
        status_label_->setText("Status: Error - No write permission for output directory");
        std::cerr << "[RecordingTab] Permission denied for directory: " << output_dir.toStdString() << std::endl;
        return;
    }
    
    // Check available disk space (require at least 1GB for meaningful recording)
    QStorageInfo storage(output_dir);
    const long long MIN_DISK_SPACE = 1073741824;  // 1GB in bytes
    if (storage.bytesFree() < MIN_DISK_SPACE) {
        char space_buf[64];
        snprintf(space_buf, sizeof(space_buf), "%.1f GB", 
                 static_cast<double>(storage.bytesFree()) / 1073741824.0);
        QString error_msg = QString::fromStdString(
            std::string("Status: Error - Insufficient disk space (") + space_buf + 
            std::string(" available, need 1GB)"));
        status_label_->setText(error_msg);
        std::cerr << "[RecordingTab] Insufficient disk space: " << space_buf << std::endl;
        return;
    }
    
    if (ros2_manager_) {
        std::cerr << "[RecordingTab] Starting recording to: " << output_dir.toStdString() << std::endl;
        bool success = ros2_manager_->start_recording(
            output_dir.toStdString(),
            {},
            compression_combo_->currentText().toStdString());
        
        if (success) {
            status_label_->setText("Status: Recording...");
            std::cerr << "[RecordingTab] Recording started successfully" << std::endl;
        } else {
            status_label_->setText("Status: Error - Failed to start recording");
            std::cerr << "[RecordingTab] Failed to start recording" << std::endl;
        }
        refresh_recording_status();
    }
}

void RecordingTab::on_stop_recording_clicked() {
    if (ros2_manager_) {
        ros2_manager_->stop_recording();
        refresh_recording_status();
    }
}

void RecordingTab::on_browse_directory_clicked() {
    QString dir = QFileDialog::getExistingDirectory(this);
    if (!dir.isEmpty()) {
        output_dir_edit_->setText(dir);
    }
}

void RecordingTab::on_refresh_bags_clicked() { refresh_bag_list(); }

void RecordingTab::on_bag_selected(int row, int column) {
    open_file_manager_button_->setEnabled(true);
    play_rviz_button_->setEnabled(true);
}

void RecordingTab::on_open_file_manager_clicked() {
    int selected_row = bags_table_->currentRow();
    if (selected_row >= 0) {
        QString output_dir = output_dir_edit_->text();
        if (!output_dir.isEmpty()) {
            std::string cmd = "xdg-open '" + output_dir.toStdString() + "' &";
            system(cmd.c_str());
        }
    }
}

void RecordingTab::on_play_in_rviz_clicked() {
    int selected_row = bags_table_->currentRow();
    if (selected_row >= 0 && bags_table_->item(selected_row, 0)) {
        QString bag_name = bags_table_->item(selected_row, 0)->text();
        QString output_dir = output_dir_edit_->text();
        QString bag_path = output_dir + "/" + bag_name;
        // Launch ros2 bag play in background
        std::string cmd = "ros2 bag play '" + bag_path.toStdString() + "' &";
        system(cmd.c_str());
    }
}

void RecordingTab::create_ui_() {
    auto layout = new QVBoxLayout();

    // Recording controls group
    auto controls_group = new QGroupBox("Recording Controls");
    auto controls_layout = new QVBoxLayout();

    auto dir_layout = new QHBoxLayout();
    output_dir_edit_ = new QLineEdit();
    output_dir_edit_->setPlaceholderText("/path/to/output");
    browse_button_ = new QPushButton("Browse...");
    connect(browse_button_, &QPushButton::clicked,
            this, &RecordingTab::on_browse_directory_clicked);
    dir_layout->addWidget(new QLabel("Output Directory:"));
    dir_layout->addWidget(output_dir_edit_);
    dir_layout->addWidget(browse_button_);

    auto format_layout = new QHBoxLayout();
    compression_combo_ = new QComboBox();
    compression_combo_->addItems({"zstd", "lz4", "none"});
    format_layout->addWidget(new QLabel("Compression:"));
    format_layout->addWidget(compression_combo_);
    format_layout->addStretch();

    auto buttons_layout = new QHBoxLayout();
    start_button_ = new QPushButton("Start Recording");
    stop_button_ = new QPushButton("Stop Recording");
    stop_button_->setEnabled(false);
    connect(start_button_, &QPushButton::clicked,
            this, &RecordingTab::on_start_recording_clicked);
    connect(stop_button_, &QPushButton::clicked,
            this, &RecordingTab::on_stop_recording_clicked);
    buttons_layout->addWidget(start_button_);
    buttons_layout->addWidget(stop_button_);
    buttons_layout->addStretch();

    auto status_layout = new QHBoxLayout();
    status_label_ = new QLabel("Status: Idle");
    elapsed_time_label_ = new QLabel("Time: 00:00:00");
    file_size_label_ = new QLabel("Size: 0 MB");
    status_layout->addWidget(status_label_);
    status_layout->addWidget(elapsed_time_label_);
    status_layout->addWidget(file_size_label_);

    controls_layout->addLayout(dir_layout);
    controls_layout->addLayout(format_layout);
    controls_layout->addLayout(buttons_layout);
    controls_layout->addLayout(status_layout);
    controls_group->setLayout(controls_layout);

    // Bag list group
    auto bags_group = new QGroupBox("Recorded Bags");
    auto bags_layout = new QVBoxLayout();
    bags_table_ = new QTableWidget();
    bags_table_->setColumnCount(5);
    bags_table_->setHorizontalHeaderLabels(
        {"Filename", "Size (MB)", "Duration", "Topics", "Date"});
    bags_table_->horizontalHeader()->setStretchLastSection(true);

    auto bags_buttons_layout = new QHBoxLayout();
    refresh_bags_button_ = new QPushButton("Refresh");
    open_file_manager_button_ = new QPushButton("Open in File Manager");
    play_rviz_button_ = new QPushButton("Play in RViz");
    open_file_manager_button_->setEnabled(false);
    play_rviz_button_->setEnabled(false);

    connect(refresh_bags_button_, &QPushButton::clicked,
            this, &RecordingTab::on_refresh_bags_clicked);
    connect(open_file_manager_button_, &QPushButton::clicked,
            this, &RecordingTab::on_open_file_manager_clicked);
    connect(play_rviz_button_, &QPushButton::clicked,
            this, &RecordingTab::on_play_in_rviz_clicked);
    connect(bags_table_, &QTableWidget::cellClicked,
            this, &RecordingTab::on_bag_selected);

    bags_buttons_layout->addWidget(refresh_bags_button_);
    bags_buttons_layout->addWidget(open_file_manager_button_);
    bags_buttons_layout->addWidget(play_rviz_button_);
    bags_buttons_layout->addStretch();

    bags_layout->addLayout(bags_buttons_layout);
    bags_layout->addWidget(bags_table_);
    bags_group->setLayout(bags_layout);

    layout->addWidget(controls_group);
    layout->addWidget(bags_group);
    setLayout(layout);
}

void RecordingTab::update_recording_controls_() {
    if (!ros2_manager_) return;
    bool recording = ros2_manager_->is_recording();
    start_button_->setEnabled(!recording);
    stop_button_->setEnabled(recording);
    auto status = ros2_manager_->get_recording_status();
    
    // Update all status labels
    status_label_->setText("Status: " + 
        QString::fromStdString(status["active"] == "true" ? "Recording" : "Idle"));
    elapsed_time_label_->setText("Time: " + 
        QString::fromStdString(status["elapsed_time"]));
    file_size_label_->setText("Size: " + 
        QString::fromStdString(status["file_size"]));
}

void RecordingTab::update_bag_list_() {
    if (!ros2_manager_) return;
    
    QString output_dir = output_dir_edit_->text();
    if (output_dir.isEmpty()) {
        bags_table_->setRowCount(0);
        return;
    }
    
    // List directory contents for bag files
    std::string cmd = "find '" + output_dir.toStdString() + "' -name 'metadata.yaml' -type f 2>/dev/null | head -20";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return;
    
    bags_table_->setRowCount(0);
    int row_count = 0;
    char buffer[512];
    
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr && row_count < 50) {
        std::string bag_path(buffer);
        bag_path.erase(std::remove(bag_path.begin(), bag_path.end(), '\n'), bag_path.end());
        
        if (bag_path.empty()) continue;
        
        // Extract bag directory name
        size_t last_slash = bag_path.rfind('/');
        if (last_slash == std::string::npos) continue;
        
        std::string bag_dir = bag_path.substr(0, last_slash);
        size_t dir_name_slash = bag_dir.rfind('/');
        std::string bag_name = (dir_name_slash != std::string::npos) ? 
            bag_dir.substr(dir_name_slash + 1) : bag_dir;
        
        if (!bag_name.empty()) {
            bags_table_->insertRow(row_count);
            
            // Filename
            auto name_item = new QTableWidgetItem(QString::fromStdString(bag_name));
            name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
            bags_table_->setItem(row_count, 0, name_item);
            
            // Size - get directory size
            std::string size_cmd = "du -sh '" + bag_dir + "' 2>/dev/null | awk '{print $1}'";
            FILE* size_pipe = popen(size_cmd.c_str(), "r");
            char size_buffer[64];
            std::string size_str = "0 MB";
            if (size_pipe && fgets(size_buffer, sizeof(size_buffer), size_pipe)) {
                size_str = size_buffer;
                size_str.erase(std::remove(size_str.begin(), size_str.end(), '\n'), size_str.end());
                if (size_pipe) pclose(size_pipe);
            }
            
            auto size_item = new QTableWidgetItem(QString::fromStdString(size_str));
            size_item->setFlags(size_item->flags() & ~Qt::ItemIsEditable);
            bags_table_->setItem(row_count, 1, size_item);
            
            // Duration - placeholder
            auto duration_item = new QTableWidgetItem("N/A");
            duration_item->setFlags(duration_item->flags() & ~Qt::ItemIsEditable);
            bags_table_->setItem(row_count, 2, duration_item);
            
            // Topics - placeholder
            auto topics_item = new QTableWidgetItem("N/A");
            topics_item->setFlags(topics_item->flags() & ~Qt::ItemIsEditable);
            bags_table_->setItem(row_count, 3, topics_item);
            
            // Date - get file modification time
            std::string date_cmd = "stat '" + bag_dir + "' -c %y 2>/dev/null | awk '{print $1}'";
            FILE* date_pipe = popen(date_cmd.c_str(), "r");
            char date_buffer[64];
            std::string date_str = "N/A";
            if (date_pipe && fgets(date_buffer, sizeof(date_buffer), date_pipe)) {
                date_str = date_buffer;
                date_str.erase(std::remove(date_str.begin(), date_str.end(), '\n'), date_str.end());
                if (date_pipe) pclose(date_pipe);
            }
            
            auto date_item = new QTableWidgetItem(QString::fromStdString(date_str));
            date_item->setFlags(date_item->flags() & ~Qt::ItemIsEditable);
            bags_table_->setItem(row_count, 4, date_item);
            
            row_count++;
        }
    }
    
    pclose(pipe);
}

}  // namespace ros2_dashboard::gui
