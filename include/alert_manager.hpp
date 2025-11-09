#pragma once

#include <QObject>
#include <QString>
#include <QDateTime>
#include <map>
#include <vector>
#include <functional>
#include <memory>
#include <mutex>

/**
 * @brief AlertManager provides intelligent alerting for ROS2 system monitoring
 * 
 * Alert types:
 * - Topic not publishing (no message for X seconds)
 * - High latency detected
 * - Message rate spike/drop
 * - Connection lost
 * - Disk space low
 * - Memory pressure
 */
class AlertManager : public QObject {
    Q_OBJECT

public:
    enum class AlertSeverity { INFO = 0, WARNING = 1, CRITICAL = 2 };

    enum class AlertType {
        TOPIC_INACTIVE,           // Topic hasn't published in X seconds
        LATENCY_HIGH,             // Message latency exceeds threshold
        RATE_ANOMALY,             // Publish rate spike or drop
        CONNECTION_LOST,          // ROS2 connection failed
        DISK_SPACE_LOW,           // Disk usage > 90%
        MEMORY_PRESSURE,          // Memory usage > 85%
        RECORDING_FAILED,         // Rosbag recording error
        NETWORK_UPLOAD_FAILED,    // Network upload error
        CUSTOM                    // User-defined alert
    };

    struct Alert {
        QString id;               // Unique alert ID
        AlertType type;
        AlertSeverity severity;
        QString topic_name;       // Related topic (if any)
        QString message;          // Alert message
        QString details;          // Additional details
        QDateTime timestamp;      // When alert was triggered
        QDateTime resolved_time;  // When alert was resolved (if any)
        bool is_active = true;    // Currently active?
        int occurrences = 1;      // How many times has this alert occurred?
    };

    static AlertManager& instance();

    /**
     * @brief Create and emit an alert
     */
    Alert create_alert(AlertType type, AlertSeverity severity,
                      const QString& message, const QString& topic_name = "",
                      const QString& details = "");

    /**
     * @brief Resolve an active alert
     */
    void resolve_alert(const QString& alert_id);

    /**
     * @brief Resolve all alerts of a specific type for a topic
     */
    void resolve_alerts_for_topic(const QString& topic_name, AlertType type);

    /**
     * @brief Check if an alert of this type already exists for this topic
     */
    bool has_active_alert(AlertType type, const QString& topic_name);

    /**
     * @brief Get all active alerts
     */
    std::vector<Alert> get_active_alerts() const;

    /**
     * @brief Get all alerts (active and resolved)
     */
    std::vector<Alert> get_all_alerts() const;

    /**
     * @brief Get alerts history
     */
    std::vector<Alert> get_alerts_history(int max_count = 100) const;

    /**
     * @brief Clear all alerts
     */
    void clear_all_alerts();

    /**
     * @brief Configure alert threshold for a specific topic
     */
    void set_topic_threshold(const QString& topic_name, AlertType type,
                            double threshold);

    /**
     * @brief Get configured threshold for topic
     */
    double get_topic_threshold(const QString& topic_name, AlertType type) const;

    /**
     * @brief Set default thresholds
     */
    void set_default_thresholds(
        double inactive_timeout_sec = 5.0,
        double latency_threshold_ms = 1000.0,
        double rate_change_percent = 50.0,
        double disk_warning_percent = 90.0,
        double memory_warning_percent = 85.0);

    /**
     * @brief Enable/disable alert aggregation (prevent spam)
     */
    void set_aggregation_enabled(bool enabled) { aggregation_enabled_ = enabled; }

    /**
     * @brief Get aggregation status
     */
    bool is_aggregation_enabled() const { return aggregation_enabled_; }

    /**
     * @brief Get alert count by severity
     */
    struct AlertStats {
        int critical_count = 0;
        int warning_count = 0;
        int info_count = 0;
        int total_count = 0;
    };

    AlertStats get_alert_statistics() const;

    /**
     * @brief Export alerts as JSON
     */
    QString export_alerts_as_json() const;

    /**
     * @brief Export alerts as CSV
     */
    QString export_alerts_as_csv() const;

signals:
    /**
     * @brief Emitted when a new alert is created
     */
    void alert_created(const Alert& alert);

    /**
     * @brief Emitted when an alert is resolved
     */
    void alert_resolved(const Alert& alert);

    /**
     * @brief Emitted when alert severity changes
     */
    void alert_severity_changed(const Alert& alert);

    /**
     * @brief Emitted for critical alerts (immediate notification)
     */
    void critical_alert(const QString& message, const QString& details);

private:
    AlertManager();

    QString generate_alert_id() const;
    QString alert_type_to_string(AlertType type) const;

    mutable std::mutex alerts_mutex_;
    std::map<QString, Alert> active_alerts_;  // By ID
    std::vector<Alert> alerts_history_;
    bool aggregation_enabled_;
    size_t max_history_size_;

    // Thresholds
    struct Thresholds {
        double inactive_timeout_sec = 5.0;
        double latency_threshold_ms = 1000.0;
        double rate_change_percent = 50.0;
        double disk_warning_percent = 90.0;
        double memory_warning_percent = 85.0;
    } default_thresholds_;

    std::map<QString, std::map<AlertType, double>> topic_thresholds_;

    // Alert deduplication
    std::map<QString, QDateTime> last_alert_time_;  // By alert key
};

