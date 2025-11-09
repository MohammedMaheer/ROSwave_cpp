#include "../include/gui/message_inspector_tab.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QSpinBox>
#include <QHeaderView>
#include <QFileDialog>
#include <QMessageBox>
#include <QDateTime>
#include <QClipboard>
#include <QApplication>
#include <QSplitter>
#include <QDebug>
#include <fstream>
#include <algorithm>
#include <iomanip>

MessageInspectorTab::MessageInspectorTab(QWidget* parent)
    : QWidget(parent), max_history_size_(100) {
    setup_ui();
    setWindowTitle("Message Inspector");
}

void MessageInspectorTab::setup_ui() {
    auto main_layout = new QVBoxLayout(this);

    // ========== Control Bar ==========
    auto control_bar = new QHBoxLayout;

    // Topic selector
    auto topic_label = new QLabel("Topic:");
    topic_selector_ = new QComboBox;
    connect(topic_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MessageInspectorTab::on_topic_selected);
    control_bar->addWidget(topic_label);
    control_bar->addWidget(topic_selector_, 1);

    control_bar->addSpacing(20);

    // Max messages spinner
    auto max_label = new QLabel("Max Messages:");
    max_messages_spin_ = new QSpinBox;
    max_messages_spin_->setMinimum(10);
    max_messages_spin_->setMaximum(1000);
    max_messages_spin_->setValue(100);
    max_messages_spin_->setSuffix(" messages");
    connect(max_messages_spin_, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &MessageInspectorTab::on_max_messages_changed);
    control_bar->addWidget(max_label);
    control_bar->addWidget(max_messages_spin_);

    // Auto-scroll checkbox
    auto auto_scroll_label = new QLabel("Auto-scroll:");
    auto_scroll_cb_ = new QCheckBox;
    auto_scroll_cb_->setChecked(true);
    connect(auto_scroll_cb_, &QCheckBox::toggled,
            this, &MessageInspectorTab::on_auto_scroll_toggled);
    control_bar->addWidget(auto_scroll_label);
    control_bar->addWidget(auto_scroll_cb_);

    // Buttons
    clear_btn_ = new QPushButton("🗑️ Clear History");
    connect(clear_btn_, &QPushButton::clicked,
            this, &MessageInspectorTab::on_clear_history_clicked);
    control_bar->addWidget(clear_btn_);

    export_btn_ = new QPushButton("💾 Export");
    connect(export_btn_, &QPushButton::clicked,
            this, &MessageInspectorTab::on_export_clicked);
    control_bar->addWidget(export_btn_);

    main_layout->addLayout(control_bar);

    // ========== Message Count & Stats ==========
    auto stats_bar = new QHBoxLayout;
    message_count_label_ = new QLabel("Messages: 0");
    frequency_label_ = new QLabel("Frequency: 0.0 Hz");
    stats_bar->addWidget(message_count_label_);
    stats_bar->addWidget(frequency_label_);
    stats_bar->addStretch();
    main_layout->addLayout(stats_bar);

    // ========== Search Bar ==========
    auto search_layout = new QHBoxLayout;
    auto search_label = new QLabel("🔍 Search:");
    search_box_ = new QLineEdit;
    search_box_->setPlaceholderText("Search in message content (JSON fields)...");
    connect(search_box_, &QLineEdit::textChanged,
            this, &MessageInspectorTab::on_search_text_changed);
    search_layout->addWidget(search_label);
    search_layout->addWidget(search_box_);
    main_layout->addLayout(search_layout);

    // ========== Main Content Area ==========
    auto splitter = new QSplitter(Qt::Horizontal);

    // Left: Message History Table
    message_history_table_ = new QTableWidget;
    message_history_table_->setColumnCount(5);
    message_history_table_->setHorizontalHeaderLabels(
        {"#", "Timestamp", "Topic", "Size (bytes)", "Preview"});
    message_history_table_->horizontalHeader()->setStretchLastSection(true);
    message_history_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    message_history_table_->setSelectionMode(QAbstractItemView::SingleSelection);
    message_history_table_->setAlternatingRowColors(true);
    connect(message_history_table_, &QTableWidget::clicked,
            this, &MessageInspectorTab::on_message_table_clicked);
    splitter->addWidget(message_history_table_);

    // Right side: Message Display & Statistics
    auto right_layout = new QVBoxLayout;

    // Message display
    auto msg_label = new QLabel("📄 Message Content (JSON):");
    message_display_ = new QTextEdit;
    message_display_->setReadOnly(true);
    message_display_->setFont(QFont("Courier New", 9));
    right_layout->addWidget(msg_label);
    right_layout->addWidget(message_display_, 2);

    // Statistics display
    auto stats_label = new QLabel("📊 Statistics:");
    statistics_display_ = new QTextEdit;
    statistics_display_->setReadOnly(true);
    statistics_display_->setFont(QFont("Courier New", 9));
    statistics_display_->setMaximumHeight(150);
    right_layout->addWidget(stats_label);
    right_layout->addWidget(statistics_display_, 1);

    auto right_widget = new QWidget;
    right_widget->setLayout(right_layout);
    splitter->addWidget(right_widget);

    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);

    main_layout->addWidget(splitter, 1);
}

void MessageInspectorTab::set_available_topics(const QStringList& topics) {
    topic_selector_->clear();
    topic_selector_->addItems(topics);
}

void MessageInspectorTab::display_message(const QString& topic_name,
                                         const QString& message_json,
                                         uint64_t timestamp) {
    if (topic_name != current_topic_) {
        return;  // Only display messages from selected topic
    }

    try {
        json msg = json::parse(message_json.toStdString());

        MessageRecord record;
        record.timestamp = timestamp;
        record.topic_name = topic_name;
        record.message_data = msg;
        record.message_size = message_json.length();
        record.received_time = QDateTime::fromMSecsSinceEpoch(timestamp);

        message_history_.push_back(record);
        if (message_history_.size() > max_history_size_) {
            message_history_.pop_front();
        }

        update_statistics();
        update_message_table();

        // Auto-scroll to latest
        if (auto_scroll_cb_->isChecked()) {
            message_history_table_->scrollToBottom();
            // Display latest message
            format_message_display(msg);
        }
    } catch (const std::exception& e) {
        qWarning() << "Error parsing message:" << e.what();
    }
}

QString MessageInspectorTab::get_selected_topic() const {
    return topic_selector_->currentText();
}

void MessageInspectorTab::clear_history() {
    message_history_.clear();
    displayed_messages_.clear();
    update_message_table();
    message_display_->clear();
    statistics_display_->clear();
}

void MessageInspectorTab::on_topic_selected(int index) {
    if (index < 0) return;

    current_topic_ = topic_selector_->currentText();
    clear_history();
    message_count_label_->setText(
        QString("Messages: 0 (Topic: %1)").arg(current_topic_));
}

void MessageInspectorTab::on_clear_history_clicked() {
    clear_history();
    QMessageBox::information(this, "Cleared", "Message history cleared.");
}

void MessageInspectorTab::on_export_clicked() {
    if (message_history_.empty()) {
        QMessageBox::warning(this, "Export", "No messages to export.");
        return;
    }

    QStringList filters;
    filters << "JSON Files (*.json)"
            << "CSV Files (*.csv)"
            << "All Files (*)";

    QString selected_filter = "JSON Files (*.json)";
    QString file_name = QFileDialog::getSaveFileName(
        this, "Export Messages", "", filters.join(";;"), &selected_filter);

    if (file_name.isEmpty()) return;

    if (file_name.endsWith(".csv")) {
        export_as_csv();
    } else {
        export_as_json();
    }

    QMessageBox::information(this, "Export Successful",
                           QString("Messages exported to:\n%1").arg(file_name));
}

void MessageInspectorTab::on_search_text_changed(const QString& search_text) {
    filter_history_by_search(search_text);
}

void MessageInspectorTab::on_message_table_clicked(const QModelIndex& index) {
    int row = index.row();
    if (row >= 0 && row < static_cast<int>(message_history_.size())) {
        const auto& record = message_history_[row];
        format_message_display(record.message_data);
    }
}

void MessageInspectorTab::on_auto_scroll_toggled(bool checked) {
    if (checked && !message_history_.empty()) {
        message_history_table_->scrollToBottom();
    }
}

void MessageInspectorTab::on_max_messages_changed(int value) {
    max_history_size_ = static_cast<size_t>(value);
    // Trim history if needed
    while (message_history_.size() > max_history_size_) {
        message_history_.pop_front();
    }
    update_message_table();
}

void MessageInspectorTab::update_message_table() {
    message_history_table_->setRowCount(0);

    int row = 0;
    for (const auto& record : message_history_) {
        message_history_table_->insertRow(row);

        // Row number
        auto item_num = new QTableWidgetItem(QString::number(row + 1));
        item_num->setFlags(item_num->flags() & ~Qt::ItemIsEditable);
        message_history_table_->setItem(row, 0, item_num);

        // Timestamp
        auto item_ts = new QTableWidgetItem(
            record.received_time.toString("hh:mm:ss.zzz"));
        item_ts->setFlags(item_ts->flags() & ~Qt::ItemIsEditable);
        message_history_table_->setItem(row, 1, item_ts);

        // Topic
        auto item_topic = new QTableWidgetItem(record.topic_name);
        item_topic->setFlags(item_topic->flags() & ~Qt::ItemIsEditable);
        message_history_table_->setItem(row, 2, item_topic);

        // Size
        auto item_size = new QTableWidgetItem(QString::number(record.message_size));
        item_size->setFlags(item_size->flags() & ~Qt::ItemIsEditable);
        message_history_table_->setItem(row, 3, item_size);

        // Preview (first field values)
        QString preview;
        try {
            for (const auto& [key, value] : record.message_data.items()) {
                if (preview.length() < 50) {
                    preview += QString::fromStdString(key) + ": ";
                    if (value.is_string()) {
                        preview += QString::fromStdString(value.get<std::string>());
                    } else if (value.is_number()) {
                        preview += QString::number(value.get<double>());
                    }
                    preview += " | ";
                }
            }
            if (preview.endsWith(" | ")) {
                preview.chop(3);
            }
        } catch (...) {
        }

        auto item_preview = new QTableWidgetItem(preview);
        item_preview->setFlags(item_preview->flags() & ~Qt::ItemIsEditable);
        message_history_table_->setItem(row, 4, item_preview);

        row++;
    }

    message_history_table_->resizeColumnsToContents();
}

void MessageInspectorTab::update_statistics() {
    if (message_history_.empty()) {
        current_stats_ = Stats();
        statistics_display_->clear();
        message_count_label_->setText("Messages: 0");
        frequency_label_->setText("Frequency: 0.0 Hz");
        return;
    }

    // Calculate statistics
    double total_size = 0;
    current_stats_.max_size_bytes = 0;
    current_stats_.min_size_bytes = std::numeric_limits<double>::max();
    current_stats_.total_messages = message_history_.size();

    for (const auto& record : message_history_) {
        total_size += record.message_size;
        current_stats_.max_size_bytes =
            std::max(current_stats_.max_size_bytes, (double)record.message_size);
        current_stats_.min_size_bytes =
            std::min(current_stats_.min_size_bytes, (double)record.message_size);

        update_field_statistics(record.message_data);
    }

    current_stats_.avg_size_bytes = total_size / message_history_.size();

    // Calculate frequency
    if (message_history_.size() > 1) {
        auto time_span_ms =
            message_history_.back().timestamp - message_history_.front().timestamp;
        if (time_span_ms > 0) {
            current_stats_.publish_rate_hz =
                (message_history_.size() - 1) * 1000.0 / time_span_ms;
        }
    }

    // Display statistics
    QString stats_text;
    stats_text += QString("📊 Message Statistics\n");
    stats_text += QString("─────────────────────\n");
    stats_text += QString("Total Messages: %1\n").arg(current_stats_.total_messages);
    stats_text +=
        QString("Publish Rate: %.2f Hz\n").arg(current_stats_.publish_rate_hz);
    stats_text += QString("Average Size: %.1f bytes\n")
                      .arg(current_stats_.avg_size_bytes);
    stats_text +=
        QString("Min Size: %.1f bytes\n").arg(current_stats_.min_size_bytes);
    stats_text +=
        QString("Max Size: %.1f bytes\n").arg(current_stats_.max_size_bytes);

    if (!current_stats_.field_ranges.empty()) {
        stats_text += QString("\n📌 Field Ranges\n");
        stats_text += QString("─────────────────────\n");
        for (const auto& [field, range] : current_stats_.field_ranges) {
            stats_text +=
                QString("%1:\n  Min: %2\n  Max: %3\n")
                    .arg(QString::fromStdString(field))
                    .arg(QString::fromStdString(range.first))
                    .arg(QString::fromStdString(range.second));
        }
    }

    statistics_display_->setText(stats_text);
    message_count_label_->setText(
        QString("Messages: %1").arg(current_stats_.total_messages));
    frequency_label_->setText(
        QString("Frequency: %.2f Hz").arg(current_stats_.publish_rate_hz));
}

void MessageInspectorTab::update_field_statistics(const json& message) {
    for (const auto& [key, value] : message.items()) {
        if (value.is_number()) {
            auto current_val = value.get<double>();
            if (current_stats_.field_ranges.find(key) ==
                current_stats_.field_ranges.end()) {
                current_stats_.field_ranges[key] = {
                    std::to_string(current_val),
                    std::to_string(current_val)};
            } else {
                auto min_val = std::stod(current_stats_.field_ranges[key].first);
                auto max_val = std::stod(current_stats_.field_ranges[key].second);
                current_stats_.field_ranges[key] = {
                    std::to_string(std::min(min_val, current_val)),
                    std::to_string(std::max(max_val, current_val))};
            }
        }
    }
}

void MessageInspectorTab::format_message_display(const json& message) {
    try {
        auto pretty_json = message.dump(2);
        message_display_->setText(QString::fromStdString(pretty_json));
    } catch (const std::exception& e) {
        message_display_->setText(QString("Error formatting message: %1").arg(e.what()));
    }
}

void MessageInspectorTab::export_as_json() {
    try {
        json export_data = json::array();
        for (const auto& record : message_history_) {
            json entry;
            entry["timestamp"] = record.timestamp;
            entry["timestamp_iso"] = record.received_time.toString(Qt::ISODate).toStdString();
            entry["topic"] = record.topic_name.toStdString();
            entry["size_bytes"] = record.message_size;
            entry["data"] = record.message_data;
            export_data.push_back(entry);
        }

        QString file_name = QFileDialog::getSaveFileName(
            this, "Export as JSON", "", "JSON Files (*.json)");
        if (!file_name.isEmpty()) {
            std::ofstream file(file_name.toStdString());
            file << export_data.dump(2);
            file.close();
        }
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Export Error",
                            QString("Failed to export: %1").arg(e.what()));
    }
}

void MessageInspectorTab::export_as_csv() {
    try {
        QString file_name =
            QFileDialog::getSaveFileName(this, "Export as CSV", "", "CSV Files (*.csv)");
        if (file_name.isEmpty()) return;

        std::ofstream file(file_name.toStdString());

        // CSV Header
        file << "Timestamp,Topic,Size (bytes),";
        if (!message_history_.empty()) {
            const auto& first_msg = message_history_.front().message_data;
            int col = 0;
            for (const auto& [key, _] : first_msg.items()) {
                if (col > 0) file << ",";
                file << key;
                col++;
            }
        }
        file << "\n";

        // CSV Data
        for (const auto& record : message_history_) {
            file << record.received_time.toString(Qt::ISODate).toStdString() << ","
                 << record.topic_name.toStdString() << "," << record.message_size << ",";

            int col = 0;
            for (const auto& [_, value] : record.message_data.items()) {
                if (col > 0) file << ",";
                if (value.is_string()) {
                    file << "\"" << value.get<std::string>() << "\"";
                } else if (value.is_number()) {
                    file << value.get<double>();
                } else if (value.is_boolean()) {
                    file << (value.get<bool>() ? "true" : "false");
                } else {
                    file << value.dump();
                }
                col++;
            }
            file << "\n";
        }

        file.close();
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Export Error",
                            QString("Failed to export: %1").arg(e.what()));
    }
}

void MessageInspectorTab::filter_history_by_search(const QString& search_text) {
    if (search_text.isEmpty()) {
        update_message_table();
        return;
    }

    // Filter messages by search text
    message_history_table_->setRowCount(0);

    int row = 0;
    for (size_t i = 0; i < message_history_.size(); ++i) {
        const auto& record = message_history_[i];
        QString msg_str = QString::fromStdString(record.message_data.dump());

        if (msg_str.contains(search_text, Qt::CaseInsensitive)) {
            message_history_table_->insertRow(row);

            auto item_num = new QTableWidgetItem(QString::number(i + 1));
            item_num->setFlags(item_num->flags() & ~Qt::ItemIsEditable);
            message_history_table_->setItem(row, 0, item_num);

            auto item_ts = new QTableWidgetItem(
                record.received_time.toString("hh:mm:ss.zzz"));
            item_ts->setFlags(item_ts->flags() & ~Qt::ItemIsEditable);
            message_history_table_->setItem(row, 1, item_ts);

            auto item_topic = new QTableWidgetItem(record.topic_name);
            item_topic->setFlags(item_topic->flags() & ~Qt::ItemIsEditable);
            message_history_table_->setItem(row, 2, item_topic);

            auto item_size = new QTableWidgetItem(QString::number(record.message_size));
            item_size->setFlags(item_size->flags() & ~Qt::ItemIsEditable);
            message_history_table_->setItem(row, 3, item_size);

            auto item_preview = new QTableWidgetItem(msg_str.left(100));
            item_preview->setFlags(item_preview->flags() & ~Qt::ItemIsEditable);
            message_history_table_->setItem(row, 4, item_preview);

            row++;
        }
    }

    message_history_table_->resizeColumnsToContents();
}

QString MessageInspectorTab::json_to_pretty_string(const json& j) const {
    return QString::fromStdString(j.dump(2));
}

