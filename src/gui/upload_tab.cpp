/**
 * @file gui/upload_tab.cpp
 */

#include "gui/upload_tab.hpp"
#include "network_manager.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTabWidget>

namespace ros2_dashboard::gui {

UploadTab::UploadTab(QWidget* parent) : QWidget(parent) { create_ui_(); }
UploadTab::~UploadTab() = default;

void UploadTab::initialize(std::shared_ptr<NetworkManager> network_manager,
                          std::shared_ptr<AsyncWorker> async_worker) {
    network_manager_ = network_manager;
    async_worker_ = async_worker;
    refresh_upload_queue();
}

void UploadTab::refresh_upload_queue() {
    if (network_manager_) {
        // Call directly in main thread (do NOT use async_worker for UI updates)
        // Qt widgets MUST be updated from the main thread only
        update_queue_display_();
    }
}

void UploadTab::on_queue_item_selected(int row, int column) {
    retry_button_->setEnabled(true);
    cancel_button_->setEnabled(true);
}

void UploadTab::on_retry_clicked() {
    int row = upload_queue_table_->currentRow();
    if (row >= 0 && network_manager_) {
        auto queue = network_manager_->get_queue();
        if (row < static_cast<int>(queue.size())) {
            network_manager_->retry_upload(queue[row].id);
            refresh_upload_queue();
        }
    }
}

void UploadTab::on_cancel_clicked() {
    int row = upload_queue_table_->currentRow();
    if (row >= 0 && network_manager_) {
        auto queue = network_manager_->get_queue();
        if (row < static_cast<int>(queue.size())) {
            network_manager_->cancel_upload(queue[row].id);
            refresh_upload_queue();
        }
    }
}

void UploadTab::on_clear_completed_clicked() {
    if (network_manager_) {
        network_manager_->clear_completed();
        refresh_upload_queue();
    }
}

void UploadTab::on_refresh_clicked() {
    refresh_upload_queue();
}

void UploadTab::create_ui_() {
    auto layout = new QVBoxLayout();

    auto tab_widget = new QTabWidget();

    // Queue tab
    auto queue_widget = new QWidget();
    auto queue_layout = new QVBoxLayout();

    upload_queue_table_ = new QTableWidget();
    upload_queue_table_->setColumnCount(6);
    upload_queue_table_->setHorizontalHeaderLabels(
        {"File", "Status", "Progress", "Size (MB)", "Priority", "Retry"});

    auto queue_buttons = new QHBoxLayout();
    retry_button_ = new QPushButton("Retry");
    cancel_button_ = new QPushButton("Cancel");
    clear_button_ = new QPushButton("Clear Completed");
    refresh_button_ = new QPushButton("Refresh");
    retry_button_->setEnabled(false);
    cancel_button_->setEnabled(false);

    connect(upload_queue_table_, &QTableWidget::cellClicked,
            this, &UploadTab::on_queue_item_selected);
    connect(retry_button_, &QPushButton::clicked,
            this, &UploadTab::on_retry_clicked);
    connect(cancel_button_, &QPushButton::clicked,
            this, &UploadTab::on_cancel_clicked);
    connect(clear_button_, &QPushButton::clicked,
            this, &UploadTab::on_clear_completed_clicked);
    connect(refresh_button_, &QPushButton::clicked,
            this, &UploadTab::on_refresh_clicked);

    queue_buttons->addWidget(retry_button_);
    queue_buttons->addWidget(cancel_button_);
    queue_buttons->addWidget(clear_button_);
    queue_buttons->addWidget(refresh_button_);
    queue_buttons->addStretch();

    queue_layout->addLayout(queue_buttons);
    queue_layout->addWidget(upload_queue_table_);
    queue_widget->setLayout(queue_layout);
    tab_widget->addTab(queue_widget, "Upload Queue");

    // Statistics
    auto stats_widget = new QWidget();
    auto stats_layout = new QVBoxLayout();
    queue_stats_label_ = new QLabel("Total: 0 | Pending: 0 | Active: 0");
    selected_upload_progress_ = new QProgressBar();
    stats_layout->addWidget(queue_stats_label_);
    stats_layout->addWidget(new QLabel("Selected Upload Progress:"));
    stats_layout->addWidget(selected_upload_progress_);
    stats_layout->addStretch();
    stats_widget->setLayout(stats_layout);
    tab_widget->addTab(stats_widget, "Statistics");

    layout->addWidget(tab_widget);
    setLayout(layout);
}

void UploadTab::update_queue_display_() {
    if (!network_manager_) return;
    auto queue = network_manager_->get_queue();
    upload_queue_table_->setRowCount(queue.size());

    for (size_t i = 0; i < queue.size(); ++i) {
        const auto& task = queue[i];
        upload_queue_table_->setItem(i, 0,
            new QTableWidgetItem(QString::fromStdString(task.file_path)));
        upload_queue_table_->setItem(i, 1,
            new QTableWidgetItem(task.status == UploadStatus::PENDING ? "Pending" :
                               task.status == UploadStatus::UPLOADING ? "Uploading" :
                               task.status == UploadStatus::COMPLETED ? "Completed" :
                               "Failed"));
    }

    auto stats = network_manager_->get_queue_stats();
    queue_stats_label_->setText(
        "Total: " + QString::fromStdString(stats["total_tasks"]) +
        " | Pending: " + QString::fromStdString(stats["pending"]) +
        " | Active: " + QString::fromStdString(stats["active_uploads"]));
}

}  // namespace ros2_dashboard::gui
