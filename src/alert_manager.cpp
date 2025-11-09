#include "../include/alert_manager.hpp"
#include <nlohmann/json.hpp>
#include <QUuid>
#include <QDebug>
#include <sstream>
#include <iomanip>

using json = nlohmann::json;

AlertManager& AlertManager::instance() {
    static AlertManager instance_;
    return instance_;
}

AlertManager::AlertManager()
    : aggregation_enabled_(true), max_history_size_(500) {
}

AlertManager::Alert AlertManager::create_alert(AlertType type, AlertSeverity severity,
                                               const QString& message,
                                               const QString& topic_name,
                                               const QString& details) {
    std::unique_lock lock(alerts_mutex_);

    // Check for duplicate alerts (aggregation)
    QString alert_key = alert_type_to_string(type) + ":" + topic_name;
    if (aggregation_enabled_) {
        auto it = last_alert_time_.find(alert_key);
        if (it != last_alert_time_.end()) {
            auto time_since_last =
                it->second.msecsTo(QDateTime::currentDateTime());
            if (time_since_last < 5000) {  // Within 5 seconds
                // Find and update existing alert
                for (auto& [id, alert] : active_alerts_) {
                    if (alert.type == type && alert.topic_name == topic_name) {
                        alert.occurrences++;
                        alert.timestamp = QDateTime::currentDateTime();
                        return alert;
                    }
                }
            }
        }
    }

    Alert alert;
    alert.id = generate_alert_id();
    alert.type = type;
    alert.severity = severity;
    alert.topic_name = topic_name;
    alert.message = message;
    alert.details = details;
    alert.timestamp = QDateTime::currentDateTime();
    alert.is_active = true;
    alert.occurrences = 1;

    active_alerts_[alert.id] = alert;
    alerts_history_.push_back(alert);

    if (alerts_history_.size() > max_history_size_) {
        alerts_history_.erase(alerts_history_.begin());
    }

    last_alert_time_[alert_key] = QDateTime::currentDateTime();

    lock.unlock();

    // Emit signals
    emit alert_created(alert);
    if (severity == AlertSeverity::CRITICAL) {
        emit critical_alert(message, details);
    }

    return alert;
}

void AlertManager::resolve_alert(const QString& alert_id) {
    std::unique_lock lock(alerts_mutex_);

    auto it = active_alerts_.find(alert_id);
    if (it != active_alerts_.end()) {
        Alert& alert = it->second;
        alert.is_active = false;
        alert.resolved_time = QDateTime::currentDateTime();

        // Move to history (already there) and remove from active
        active_alerts_.erase(it);

        lock.unlock();
        emit alert_resolved(alert);
    }
}

void AlertManager::resolve_alerts_for_topic(const QString& topic_name,
                                            AlertType type) {
    std::unique_lock lock(alerts_mutex_);

    auto it = active_alerts_.begin();
    while (it != active_alerts_.end()) {
        if (it->second.topic_name == topic_name && it->second.type == type) {
            Alert alert = it->second;
            alert.is_active = false;
            alert.resolved_time = QDateTime::currentDateTime();
            it = active_alerts_.erase(it);
            lock.unlock();
            emit alert_resolved(alert);
            lock.lock();
        } else {
            ++it;
        }
    }
}

bool AlertManager::has_active_alert(AlertType type, const QString& topic_name) {
    std::unique_lock lock(alerts_mutex_);

    for (const auto& [_, alert] : active_alerts_) {
        if (alert.type == type && alert.topic_name == topic_name &&
            alert.is_active) {
            return true;
        }
    }
    return false;
}

std::vector<AlertManager::Alert> AlertManager::get_active_alerts() const {
    std::unique_lock lock(alerts_mutex_);

    std::vector<Alert> result;
    for (const auto& [_, alert] : active_alerts_) {
        if (alert.is_active) {
            result.push_back(alert);
        }
    }
    return result;
}

std::vector<AlertManager::Alert> AlertManager::get_all_alerts() const {
    std::unique_lock lock(alerts_mutex_);
    std::vector<Alert> result;
    for (const auto& [_, alert] : active_alerts_) {
        result.push_back(alert);
    }
    return result;
}

std::vector<AlertManager::Alert> AlertManager::get_alerts_history(
    int max_count) const {
    std::unique_lock lock(alerts_mutex_);

    std::vector<Alert> result;
    int start_idx =
        std::max(0, (int)alerts_history_.size() - max_count);
    for (int i = start_idx; i < (int)alerts_history_.size(); ++i) {
        result.push_back(alerts_history_[i]);
    }
    return result;
}

void AlertManager::clear_all_alerts() {
    std::unique_lock lock(alerts_mutex_);
    active_alerts_.clear();
    alerts_history_.clear();
    last_alert_time_.clear();
}

void AlertManager::set_topic_threshold(const QString& topic_name, AlertType type,
                                       double threshold) {
    std::unique_lock lock(alerts_mutex_);
    topic_thresholds_[topic_name][type] = threshold;
}

double AlertManager::get_topic_threshold(const QString& topic_name,
                                         AlertType type) const {
    std::unique_lock lock(alerts_mutex_);

    auto it = topic_thresholds_.find(topic_name);
    if (it != topic_thresholds_.end()) {
        auto type_it = it->second.find(type);
        if (type_it != it->second.end()) {
            return type_it->second;
        }
    }

    // Return default based on type
    switch (type) {
        case AlertType::TOPIC_INACTIVE:
            return default_thresholds_.inactive_timeout_sec;
        case AlertType::LATENCY_HIGH:
            return default_thresholds_.latency_threshold_ms;
        case AlertType::RATE_ANOMALY:
            return default_thresholds_.rate_change_percent;
        case AlertType::DISK_SPACE_LOW:
            return default_thresholds_.disk_warning_percent;
        case AlertType::MEMORY_PRESSURE:
            return default_thresholds_.memory_warning_percent;
        default:
            return 0.0;
    }
}

void AlertManager::set_default_thresholds(
    double inactive_timeout_sec, double latency_threshold_ms,
    double rate_change_percent, double disk_warning_percent,
    double memory_warning_percent) {
    std::unique_lock lock(alerts_mutex_);

    default_thresholds_.inactive_timeout_sec = inactive_timeout_sec;
    default_thresholds_.latency_threshold_ms = latency_threshold_ms;
    default_thresholds_.rate_change_percent = rate_change_percent;
    default_thresholds_.disk_warning_percent = disk_warning_percent;
    default_thresholds_.memory_warning_percent = memory_warning_percent;
}

AlertManager::AlertStats AlertManager::get_alert_statistics() const {
    std::unique_lock lock(alerts_mutex_);

    AlertStats stats;
    for (const auto& [_, alert] : active_alerts_) {
        if (alert.is_active) {
            stats.total_count++;
            switch (alert.severity) {
                case AlertSeverity::CRITICAL:
                    stats.critical_count++;
                    break;
                case AlertSeverity::WARNING:
                    stats.warning_count++;
                    break;
                case AlertSeverity::INFO:
                    stats.info_count++;
                    break;
            }
        }
    }
    return stats;
}

QString AlertManager::export_alerts_as_json() const {
    std::unique_lock lock(alerts_mutex_);

    json alerts_array = json::array();
    for (const auto& alert : alerts_history_) {
        json alert_obj;
        alert_obj["id"] = alert.id.toStdString();
        alert_obj["type"] = alert_type_to_string(alert.type).toStdString();
        alert_obj["severity"] = static_cast<int>(alert.severity);
        alert_obj["topic"] = alert.topic_name.toStdString();
        alert_obj["message"] = alert.message.toStdString();
        alert_obj["details"] = alert.details.toStdString();
        alert_obj["timestamp"] = alert.timestamp.toString(Qt::ISODate).toStdString();
        alert_obj["is_active"] = alert.is_active;
        alert_obj["occurrences"] = alert.occurrences;

        if (!alert.resolved_time.isNull()) {
            alert_obj["resolved_time"] =
                alert.resolved_time.toString(Qt::ISODate).toStdString();
        }

        alerts_array.push_back(alert_obj);
    }

    return QString::fromStdString(alerts_array.dump(2));
}

QString AlertManager::export_alerts_as_csv() const {
    std::unique_lock lock(alerts_mutex_);

    std::stringstream ss;
    ss << "ID,Type,Severity,Topic,Message,Details,Timestamp,Is Active,Occurrences\n";

    for (const auto& alert : alerts_history_) {
        ss << alert.id.toStdString() << ","
           << alert_type_to_string(alert.type).toStdString() << ","
           << static_cast<int>(alert.severity) << ","
           << alert.topic_name.toStdString() << ","
           << "\"" << alert.message.toStdString() << "\","
           << "\"" << alert.details.toStdString() << "\","
           << alert.timestamp.toString(Qt::ISODate).toStdString() << ","
           << (alert.is_active ? "Yes" : "No") << ","
           << alert.occurrences << "\n";
    }

    return QString::fromStdString(ss.str());
}

QString AlertManager::generate_alert_id() const {
    return QUuid::createUuid().toString();
}

QString AlertManager::alert_type_to_string(AlertType type) const {
    switch (type) {
        case AlertType::TOPIC_INACTIVE:
            return "Topic Inactive";
        case AlertType::LATENCY_HIGH:
            return "High Latency";
        case AlertType::RATE_ANOMALY:
            return "Rate Anomaly";
        case AlertType::CONNECTION_LOST:
            return "Connection Lost";
        case AlertType::DISK_SPACE_LOW:
            return "Low Disk Space";
        case AlertType::MEMORY_PRESSURE:
            return "Memory Pressure";
        case AlertType::RECORDING_FAILED:
            return "Recording Failed";
        case AlertType::NETWORK_UPLOAD_FAILED:
            return "Upload Failed";
        case AlertType::CUSTOM:
            return "Custom Alert";
        default:
            return "Unknown";
    }
}

