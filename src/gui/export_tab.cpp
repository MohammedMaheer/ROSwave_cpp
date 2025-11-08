/**
 * @file gui/export_tab.cpp
 */

#include "gui/export_tab.hpp"
#include "ml_exporter.hpp"
#include "ros2_manager.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QProgressBar>

namespace ros2_dashboard::gui {

ExportTab::ExportTab(QWidget* parent) : QWidget(parent) { create_ui_(); }
ExportTab::~ExportTab() = default;

void ExportTab::initialize(std::shared_ptr<MLExporter> ml_exporter,
                          std::shared_ptr<ROS2Manager> ros2_manager,
                          std::shared_ptr<AsyncWorker> async_worker) {
    ml_exporter_ = ml_exporter;
    ros2_manager_ = ros2_manager;
    async_worker_ = async_worker;
}

void ExportTab::on_add_bags_clicked() {
    QStringList files = QFileDialog::getOpenFileNames(this,
        "Select bag files", "", "ROS2 Bags (*)");
    for (const auto& file : files) {
        bags_list_->addItem(file);
    }
}

void ExportTab::on_remove_selected_clicked() {
    delete bags_list_->takeItem(bags_list_->currentRow());
}

void ExportTab::on_export_clicked() {
    if (ml_exporter_) {
        ExportConfig config;
        config.output_directory = output_dir_edit_->text().toStdString();
        config.metadata_annotation = metadata_edit_->toPlainText().toStdString();
        for (int i = 0; i < bags_list_->count(); ++i) {
            config.bag_files.push_back(bags_list_->item(i)->text().toStdString());
        }

        async_worker_->enqueue([this, config]() {
            ml_exporter_->export_batch(config, [this](int progress, const auto&) {
                export_progress_->setValue(progress);
            });
        });
    }
}

void ExportTab::on_output_dir_browse_clicked() {
    QString dir = QFileDialog::getExistingDirectory(this);
    if (!dir.isEmpty()) {
        output_dir_edit_->setText(dir);
    }
}

void ExportTab::create_ui_() {
    auto layout = new QVBoxLayout();

    auto buttons_layout = new QHBoxLayout();
    add_bags_button_ = new QPushButton("Add Bags");
    remove_button_ = new QPushButton("Remove");
    connect(add_bags_button_, &QPushButton::clicked,
            this, &ExportTab::on_add_bags_clicked);
    connect(remove_button_, &QPushButton::clicked,
            this, &ExportTab::on_remove_selected_clicked);
    buttons_layout->addWidget(add_bags_button_);
    buttons_layout->addWidget(remove_button_);
    buttons_layout->addStretch();

    bags_list_ = new QListWidget();

    auto dir_layout = new QHBoxLayout();
    auto browse_button = new QPushButton("Browse...");
    output_dir_edit_ = new QLineEdit();
    output_dir_edit_->setPlaceholderText("/path/to/output");
    connect(browse_button, &QPushButton::clicked,
            this, &ExportTab::on_output_dir_browse_clicked);
    dir_layout->addWidget(new QLabel("Output Directory:"));
    dir_layout->addWidget(output_dir_edit_);
    dir_layout->addWidget(browse_button);

    metadata_edit_ = new QPlainTextEdit();
    metadata_edit_->setPlaceholderText("Optional metadata annotation...");
    metadata_edit_->setMaximumHeight(80);

    export_button_ = new QPushButton("Export");
    connect(export_button_, &QPushButton::clicked,
            this, &ExportTab::on_export_clicked);

    export_progress_ = new QProgressBar();
    export_progress_->setMaximum(100);

    layout->addLayout(buttons_layout);
    layout->addWidget(bags_list_);
    layout->addLayout(dir_layout);
    layout->addWidget(new QLabel("Metadata:"));
    layout->addWidget(metadata_edit_);
    layout->addWidget(export_button_);
    layout->addWidget(export_progress_);
    setLayout(layout);
}

}  // namespace ros2_dashboard::gui
