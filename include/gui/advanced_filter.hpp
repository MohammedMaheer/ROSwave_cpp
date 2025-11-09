#pragma once

#include <QWidget>
#include <QLineEdit>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QStringListModel>
#include <QString>
#include <QStringList>
#include <QVector>
#include <QMap>
#include <QRegularExpression>
#include <memory>

/**
 * @brief Advanced search and filtering system for topics and data
 * 
 * Features:
 * - Real-time incremental search
 * - Regex pattern matching (prefix with 'r:')
 * - Filter by: name, type, rate, status
 * - Save filter presets
 * - Search history with autocomplete
 * - Cross-tab search
 */
class AdvancedFilterWidget : public QWidget {
    Q_OBJECT

public:
    explicit AdvancedFilterWidget(QWidget* parent = nullptr);

    /**
     * @brief Filter criteria for queries
     */
    struct FilterCriteria {
        QString search_text;
        QString filter_field;  // "All Fields", "Name", "Type", "Rate", "Status"
        bool use_regex = false;
        double min_rate = 0.0;
        double max_rate = 1e6;
        QString status_filter;  // "All", "Active", "Inactive", "Error"
    };

    /**
     * @brief Get current filter criteria
     */
    FilterCriteria get_filter_criteria() const;

    /**
     * @brief Set available filter fields
     */
    void set_filter_fields(const QStringList& fields);

    /**
     * @brief Add item to search history
     */
    void add_to_search_history(const QString& query);

    /**
     * @brief Get search history
     */
    QStringList get_search_history() const;

    /**
     * @brief Clear search history
     */
    void clear_search_history();

    /**
     * @brief Save filter as preset
     */
    void save_filter_preset(const QString& preset_name, const FilterCriteria& criteria);

    /**
     * @brief Load filter preset
     */
    bool load_filter_preset(const QString& preset_name);

    /**
     * @brief Get available presets
     */
    QStringList get_filter_presets() const;

    /**
     * @brief Delete filter preset
     */
    void delete_filter_preset(const QString& preset_name);

    /**
     * @brief Clear all filters
     */
    void clear_filters();

    /**
     * @brief Enable cross-tab search
     */
    void set_cross_tab_search_enabled(bool enabled);

    /**
     * @brief Match text against criteria
     */
    bool matches_criteria(const QString& text, const FilterCriteria& criteria) const;

signals:
    /**
     * @brief Emitted when filter criteria changes
     */
    void filter_changed(const FilterCriteria& criteria);

    /**
     * @brief Emitted when search is executed
     */
    void search_executed(const QString& query);

    /**
     * @brief Emitted when preset is selected
     */
    void preset_selected(const QString& preset_name);

private slots:
    void on_search_text_changed(const QString& text);
    void on_filter_field_changed(int index);
    void on_regex_toggled(bool checked);
    void on_min_rate_changed(double value);
    void on_max_rate_changed(double value);
    void on_status_changed(int index);
    void on_clear_clicked();
    void on_preset_selected(int index);
    void on_save_preset_clicked();
    void on_delete_preset_clicked();

private:
    void setup_ui();
    void update_criteria();
    bool validate_regex_pattern(const QString& pattern) const;

    // UI Components
    QLineEdit* search_input_;
    QComboBox* filter_field_combo_;
    QCheckBox* regex_checkbox_;
    QDoubleSpinBox* min_rate_spin_;
    QDoubleSpinBox* max_rate_spin_;
    QComboBox* status_combo_;
    QPushButton* clear_btn_;
    QPushButton* save_preset_btn_;
    QPushButton* delete_preset_btn_;
    QComboBox* preset_combo_;

    // Data
    FilterCriteria current_criteria_;
    QStringList search_history_;
    QMap<QString, FilterCriteria> filter_presets_;
    bool cross_tab_search_enabled_;
    static constexpr int MAX_HISTORY_SIZE = 20;
};

