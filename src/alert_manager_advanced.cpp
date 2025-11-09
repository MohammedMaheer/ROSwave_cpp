/**
 * @file alert_manager_advanced.cpp
 * @brief Advanced alerting system implementation
 */

#include "alert_manager_advanced.hpp"
#include <iostream>
#include <algorithm>
#include <ctime>

namespace ros2_dashboard {

AlertManager::AlertManager() {
    std::cerr << "[AlertManager] Initialized advanced alert system" << std::endl;
}

void AlertManager::raise_alert(const Alert& alert) {
    std::lock_guard<std::mutex> lock(mtx_);
    
    // Check for deduplication
    if (should_dedupe_(alert)) {
        std::cerr << "[AlertManager] Deduplicating alert: " << alert.message << std::endl;
        return;
    }
    
    auto mutable_alert = const_cast<Alert&>(alert);
    mutable_alert.timestamp_ms = std::time(nullptr) * 1000;
    
    active_alerts_.push_back(mutable_alert);
    alert_history_.push_back(mutable_alert);
    
    // Maintain history size
    if (alert_history_.size() > MAX_HISTORY) {
        alert_history_.erase(alert_history_.begin());
    }
    
    // Log alert
    std::cerr << "[AlertManager] Alert raised [" 
              << (int)alert.severity << "]: " << alert.message << std::endl;
    
    // Trigger callbacks
    trigger_callbacks_(mutable_alert);
}

void AlertManager::raise_topic_alert(AlertType type, const std::string& topic_name,
                                     AlertSeverity severity, const std::string& message) {
    Alert alert;
    alert.type = type;
    alert.severity = severity;
    alert.topic_name = topic_name;
    alert.message = message;
    alert.component = "TopicMonitor";
    
    raise_alert(alert);
}

void AlertManager::raise_system_alert(AlertType type, AlertSeverity severity,
                                     const std::string& message) {
    Alert alert;
    alert.type = type;
    alert.severity = severity;
    alert.message = message;
    alert.component = "System";
    
    raise_alert(alert);
}

void AlertManager::set_threshold(const AlertThreshold& threshold) {
    std::lock_guard<std::mutex> lock(mtx_);
    
    // Remove existing threshold if present
    auto it = std::find_if(thresholds_.begin(), thresholds_.end(),
        [&](const AlertThreshold& t) {
            return t.topic_name == threshold.topic_name && t.alert_type == threshold.alert_type;
        });
    
    if (it != thresholds_.end()) {
        thresholds_.erase(it);
    }
    
    thresholds_.push_back(threshold);
    std::cerr << "[AlertManager] Threshold set for " << threshold.topic_name << std::endl;
}

void AlertManager::remove_threshold(const std::string& topic_name, AlertType type) {
    std::lock_guard<std::mutex> lock(mtx_);
    
    auto it = std::find_if(thresholds_.begin(), thresholds_.end(),
        [&](const AlertThreshold& t) {
            return t.topic_name == topic_name && t.alert_type == type;
        });
    
    if (it != thresholds_.end()) {
        thresholds_.erase(it);
        std::cerr << "[AlertManager] Threshold removed for " << topic_name << std::endl;
    }
}

std::vector<AlertThreshold> AlertManager::get_thresholds() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return thresholds_;
}

AlertThreshold* AlertManager::get_threshold(const std::string& topic_name, AlertType type) {
    std::lock_guard<std::mutex> lock(mtx_);
    
    auto it = std::find_if(thresholds_.begin(), thresholds_.end(),
        [&](const AlertThreshold& t) {
            return t.topic_name == topic_name && t.alert_type == type;
        });
    
    if (it != thresholds_.end()) {
        return &(*it);
    }
    return nullptr;
}

std::vector<Alert> AlertManager::get_active_alerts() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return active_alerts_;
}

std::vector<Alert> AlertManager::get_alert_history(uint32_t limit) const {
    std::lock_guard<std::mutex> lock(mtx_);
    
    std::vector<Alert> result;
    size_t start = (alert_history_.size() > limit) ? (alert_history_.size() - limit) : 0;
    result.insert(result.end(), alert_history_.begin() + start, alert_history_.end());
    return result;
}

std::vector<Alert> AlertManager::get_alerts_for_topic(const std::string& topic_name) const {
    std::lock_guard<std::mutex> lock(mtx_);
    
    std::vector<Alert> result;
    for (const auto& alert : active_alerts_) {
        if (alert.topic_name == topic_name) {
            result.push_back(alert);
        }
    }
    return result;
}

std::vector<Alert> AlertManager::get_alerts_by_severity(AlertSeverity severity) const {
    std::lock_guard<std::mutex> lock(mtx_);
    
    std::vector<Alert> result;
    for (const auto& alert : active_alerts_) {
        if (alert.severity == severity) {
            result.push_back(alert);
        }
    }
    return result;
}

void AlertManager::acknowledge_alert(size_t alert_id) {
    std::lock_guard<std::mutex> lock(mtx_);
    
    if (alert_id < active_alerts_.size()) {
        active_alerts_[alert_id].auto_resolved = true;
        active_alerts_[alert_id].resolution_time_ms = std::time(nullptr) * 1000;
        std::cerr << "[AlertManager] Alert acknowledged" << std::endl;
    }
}

void AlertManager::clear_resolved_alerts() {
    std::lock_guard<std::mutex> lock(mtx_);
    
    auto it = std::remove_if(active_alerts_.begin(), active_alerts_.end(),
        [](const Alert& a) { return a.auto_resolved; });
    
    active_alerts_.erase(it, active_alerts_.end());
    std::cerr << "[AlertManager] Cleared resolved alerts" << std::endl;
}

void AlertManager::clear_all_alerts() {
    std::lock_guard<std::mutex> lock(mtx_);
    active_alerts_.clear();
    std::cerr << "[AlertManager] Cleared all alerts" << std::endl;
}

void AlertManager::on_alert(AlertCallback callback) {
    std::lock_guard<std::mutex> lock(mtx_);
    alert_callbacks_.push_back(callback);
}

void AlertManager::on_severity_change(AlertCallback callback) {
    std::lock_guard<std::mutex> lock(mtx_);
    severity_change_callbacks_.push_back(callback);
}

AlertManager::AlertStats AlertManager::get_statistics() const {
    std::lock_guard<std::mutex> lock(mtx_);
    
    AlertStats stats;
    stats.total_alerts_raised = alert_history_.size();
    stats.active_alerts_count = active_alerts_.size();
    
    double total_resolution_time = 0;
    int resolved_count = 0;
    
    for (const auto& alert : alert_history_) {
        if (alert.auto_resolved) {
            total_resolution_time += alert.resolution_time_ms - alert.timestamp_ms;
            resolved_count++;
        }
        
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
    
    if (resolved_count > 0) {
        stats.avg_resolution_time_ms = total_resolution_time / resolved_count;
    }
    
    return stats;
}

void AlertManager::check_topic_metrics(const std::string& topic_name,
                                       double message_rate,
                                       double latency_ms,
                                       uint32_t dropped_messages) {
    std::lock_guard<std::mutex> lock(mtx_);
    
    // Check each threshold for this topic
    for (const auto& threshold : thresholds_) {
        if (!threshold.enabled || threshold.topic_name != topic_name) {
            continue;
        }
        
        bool condition_met = false;
        std::string message;
        
        switch (threshold.alert_type) {
            case AlertType::TOPIC_HIGH_LATENCY:
                if (latency_ms > threshold.threshold_value) {
                    condition_met = true;
                    message = "Latency " + std::to_string(static_cast<int>(latency_ms)) + 
                             "ms exceeds threshold " + std::to_string(static_cast<int>(threshold.threshold_value)) + "ms";
                }
                break;
                
            case AlertType::TOPIC_HIGH_RATE:
                if (message_rate > threshold.threshold_value) {
                    condition_met = true;
                    message = "Message rate " + std::to_string(static_cast<int>(message_rate)) + 
                             "/s exceeds threshold " + std::to_string(static_cast<int>(threshold.threshold_value)) + "/s";
                }
                break;
                
            case AlertType::TOPIC_DROPPED_MESSAGES:
                if (dropped_messages > threshold.threshold_value) {
                    condition_met = true;
                    message = std::to_string(dropped_messages) + " messages dropped";
                }
                break;
                
            default:
                break;
        }
        
        if (condition_met) {
            Alert alert;
            alert.type = threshold.alert_type;
            alert.severity = threshold.severity;
            alert.topic_name = topic_name;
            alert.message = message;
            alert.component = "MetricsChecker";
            alert.timestamp_ms = std::time(nullptr) * 1000;
            
            // This will be deduplicated if it occurred recently
            if (!should_dedupe_(alert)) {
                active_alerts_.push_back(alert);
                alert_history_.push_back(alert);
                trigger_callbacks_(alert);
            }
        }
    }
}

void AlertManager::reset() {
    std::lock_guard<std::mutex> lock(mtx_);
    active_alerts_.clear();
    alert_history_.clear();
    thresholds_.clear();
    last_alert_time_.clear();
    std::cerr << "[AlertManager] Reset all alerts" << std::endl;
}

std::string AlertManager::alert_key_(const Alert& alert) const {
    return alert.topic_name + "|" + std::to_string(static_cast<int>(alert.type));
}

bool AlertManager::should_dedupe_(const Alert& alert) {
    auto key = alert_key_(alert);
    auto now = std::time(nullptr) * 1000;
    
    auto it = last_alert_time_.find(key);
    if (it != last_alert_time_.end()) {
        if (now - it->second < ALERT_DEDUPE_WINDOW_MS) {
            return true;  // Deduplicate
        }
    }
    
    last_alert_time_[key] = now;
    return false;
}

void AlertManager::trigger_callbacks_(const Alert& alert) {
    for (auto& callback : alert_callbacks_) {
        try {
            callback(alert);
        } catch (const std::exception& e) {
            std::cerr << "[AlertManager] Callback error: " << e.what() << std::endl;
        }
    }
}

}  // namespace ros2_dashboard
