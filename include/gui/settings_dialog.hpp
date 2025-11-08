/**
 * @file gui/settings_dialog.hpp
 * @brief Application settings and configuration dialog
 * @author Dashboard Team
 */

#pragma once

#include <QDialog>
#include <QLineEdit>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QJsonDocument>
#include <QJsonObject>
#include <memory>

namespace ros2_dashboard::gui {

/**
 * @class SettingsDialog
 * @brief Application settings configuration
 */
class SettingsDialog : public QDialog {
    Q_OBJECT

public:
    explicit SettingsDialog(QWidget* parent = nullptr);
    ~SettingsDialog() override;

private slots:
    void on_apply_clicked();
    void on_reset_clicked();

private:
    // Cache settings
    QSpinBox* cache_ttl_spin_;

    // Update intervals
    QSpinBox* topic_update_interval_spin_;
    QSpinBox* node_update_interval_spin_;
    QSpinBox* metrics_update_interval_spin_;

    // Thread pool
    QSpinBox* thread_pool_size_spin_;

    // Network settings
    QLineEdit* upload_endpoint_edit_;
    QLineEdit* api_key_edit_;
    QSpinBox* chunk_size_spin_;
    QSpinBox* max_concurrent_uploads_spin_;
    QDoubleSpinBox* bandwidth_limit_spin_;

    // Recording settings
    QLineEdit* recording_dir_edit_;

    // UI settings
    QCheckBox* dark_theme_checkbox_;
    QCheckBox* debug_logging_checkbox_;

    QPushButton* apply_button_;
    QPushButton* reset_button_;
    QPushButton* ok_button_;
    QPushButton* cancel_button_;

    void create_ui_();
    void load_settings_();
    void save_settings_();
};

}  // namespace ros2_dashboard::gui
