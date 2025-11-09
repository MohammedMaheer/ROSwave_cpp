#pragma once

#include <QWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QLineEdit>
#include <QCheckBox>
#include <QDateTime>
#include <deque>
#include <memory>
#include <map>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * @brief MessageInspectorTab displays live message content from ROS2 topics
 * 
 * Features:
 * - Real-time message display in JSON format
 * - Message history (last 100 messages)
 * - Message statistics (size, frequency, fields)
 * - Field-level breakdown with value ranges
 * - Search within message history
 * - Export messages as JSON/CSV
 */
class MessageInspectorTab : public QWidget {
    Q_OBJECT

public:
    explicit MessageInspectorTab(QWidget* parent = nullptr);
    ~MessageInspectorTab() = default;

    /**
     * @brief Populate available topics for inspection
     */
    void set_available_topics(const QStringList& topics);

    /**
     * @brief Display a new message from selected topic
     * @param topic_name Topic name
     * @param message_json JSON representation of message
     * @param timestamp Message timestamp in milliseconds
     */
    void display_message(const QString& topic_name, const QString& message_json,
                        uint64_t timestamp);

    /**
     * @brief Get currently selected topic
     */
    QString get_selected_topic() const;

    /**
     * @brief Clear all message history
     */
    void clear_history();

private slots:
    void on_topic_selected(int index);
    void on_clear_history_clicked();
    void on_export_clicked();
    void on_search_text_changed(const QString& text);
    void on_message_table_clicked(const QModelIndex& index);
    void on_auto_scroll_toggled(bool checked);
    void on_max_messages_changed(int value);

private:
    struct MessageRecord {
        uint64_t timestamp;        // milliseconds since epoch
        QString topic_name;
        json message_data;
        int message_size;          // bytes
        QDateTime received_time;
    };

    // UI Components
    QComboBox* topic_selector_;
    QTableWidget* message_history_table_;
    QTextEdit* message_display_;
    QTextEdit* statistics_display_;
    QLineEdit* search_box_;
    QPushButton* clear_btn_;
    QPushButton* export_btn_;
    QCheckBox* auto_scroll_cb_;
    QSpinBox* max_messages_spin_;
    QLabel* message_count_label_;
    QLabel* frequency_label_;

    // Data storage
    std::deque<MessageRecord> message_history_;
    size_t max_history_size_;
    QString current_topic_;
    std::vector<QString> displayed_messages_;  // for search filtering

    // Statistics
    struct Stats {
        double avg_size_bytes = 0;
        double max_size_bytes = 0;
        double min_size_bytes = 0;
        double publish_rate_hz = 0;
        int total_messages = 0;
        std::map<std::string, std::pair<std::string, std::string>> field_ranges;
    } current_stats_;

    // Helper methods
    void setup_ui();
    void update_message_table();
    void update_statistics();
    void update_field_statistics(const json& message);
    void format_message_display(const json& message);
    void export_as_json();
    void export_as_csv();
    void filter_history_by_search(const QString& search_text);
    QString json_to_pretty_string(const json& j) const;
};

