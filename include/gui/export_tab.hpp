/**
 * @file gui/export_tab.hpp
 * @brief Data export and ML dataset packaging tab
 * @author Dashboard Team
 */

#pragma once

#include <QWidget>
#include <QListWidget>
#include <QPushButton>
#include <QProgressBar>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <memory>
#include "ml_exporter.hpp"
#include "ros2_manager.hpp"
#include "async_worker.hpp"

namespace ros2_dashboard::gui {

/**
 * @class ExportTab
 * @brief Manages ML dataset export and packaging
 */
class ExportTab : public QWidget {
    Q_OBJECT

public:
    explicit ExportTab(QWidget* parent = nullptr);
    ~ExportTab() override;

    void initialize(std::shared_ptr<MLExporter> ml_exporter,
                   std::shared_ptr<ROS2Manager> ros2_manager,
                   std::shared_ptr<AsyncWorker> async_worker);

private slots:
    void on_add_bags_clicked();
    void on_remove_selected_clicked();
    void on_export_clicked();
    void on_output_dir_browse_clicked();

private:
    std::shared_ptr<MLExporter> ml_exporter_;
    std::shared_ptr<ROS2Manager> ros2_manager_;
    std::shared_ptr<AsyncWorker> async_worker_;

    QListWidget* bags_list_;
    QPushButton* add_bags_button_;
    QPushButton* remove_button_;
    QLineEdit* output_dir_edit_;
    QPlainTextEdit* metadata_edit_;
    QPushButton* export_button_;
    QProgressBar* export_progress_;

    void create_ui_();
};

}  // namespace ros2_dashboard::gui
