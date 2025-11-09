/**
 * @file analytics_dashboard.hpp
 * @brief Advanced Analytics Dashboard for trend analysis, predictions, and statistical reports
 * @author Dashboard Team
 */

#pragma once

#include <vector>
#include <string>
#include <map>
#include <memory>
#include <cstdint>
#include "metrics_history_buffer.hpp"
#include "advanced_chart_features.hpp"
#include <nlohmann/json.hpp>

namespace ros2_dashboard {

/**
 * @struct CorrelationPair
 * @brief Represents correlation between two metrics
 */
struct CorrelationPair {
    std::string metric1;
    std::string metric2;
    double correlation;  ///< Range -1.0 to 1.0
    std::string relationship;  ///< "strong_positive", "weak_positive", "no_correlation", etc.
};

/**
 * @struct AnomalyReport
 * @brief Report on detected anomalies in metrics
 */
struct AnomalyReport {
    std::string metric_name;
    std::vector<charts::AnomalyInfo> anomalies;
    size_t total_points;
    double anomaly_rate;  ///< Percentage of points marked as anomalies
    std::string severity;  ///< "info", "warning", "critical"
};

/**
 * @struct PredictionReport
 * @brief Report on trend predictions
 */
struct PredictionReport {
    std::string metric_name;
    charts::PredictionInfo prediction;
    std::vector<double> actual_values;
    std::vector<double> predicted_values;
    double accuracy_percent;  ///< Prediction accuracy vs actual data
    std::string recommendation;  ///< Action recommendation based on prediction
};

/**
 * @struct StatisticalReport
 * @brief Comprehensive statistical report for all metrics
 */
struct StatisticalReport {
    uint64_t report_time_ms;
    std::string time_window;  ///< "last_5min", "last_hour", "last_day"
    
    std::map<std::string, charts::ChartAnalytics::Stats> metric_stats;
    std::vector<CorrelationPair> correlations;
    std::vector<AnomalyReport> anomalies;
    std::vector<PredictionReport> predictions;
    
    struct SummaryMetrics {
        double overall_health_percent;  ///< 0-100
        std::string status;  ///< "healthy", "degraded", "critical"
        int alert_count;
        int anomaly_count;
        std::string primary_concern;  ///< Most critical issue
    };
    SummaryMetrics summary;
};

/**
 * @struct TrendAnalysis
 * @brief Time-series trend analysis results
 */
struct TrendAnalysis {
    std::string metric_name;
    std::vector<uint64_t> timestamps;
    std::vector<double> values;
    
    // Trend characteristics
    double slope;  ///< Rate of change over time
    double trend_strength;  ///< 0-1, how pronounced the trend is
    std::string direction;  ///< "up", "down", "stable"
    
    // Seasonal/cyclic patterns
    bool has_periodicity;
    int period_minutes;  ///< Estimated period if periodic
    
    // Recent change
    double change_last_hour;
    double percent_change_last_hour;
};

/**
 * @struct PerformanceMetrics
 * @brief Performance characteristics of system
 */
struct PerformanceMetrics {
    // Response times
    struct ResponseTime {
        double min_ms;
        double max_ms;
        double avg_ms;
        double p95_ms;
        double p99_ms;
    };
    
    ResponseTime cpu_response;
    ResponseTime memory_response;
    ResponseTime disk_response;
    ResponseTime network_response;
    
    // Throughput
    struct Throughput {
        double min_ops;
        double max_ops;
        double avg_ops;
        double p95_ops;
    };
    
    Throughput cpu_throughput;
    Throughput network_throughput;
    
    // Bottlenecks
    std::vector<std::string> identified_bottlenecks;
    std::vector<std::string> optimization_suggestions;
};

/**
 * @class AnalyticsDashboard
 * @brief Complete analytics system for historical data analysis and reporting
 */
class AnalyticsDashboard {
public:
    /**
     * @brief Constructor
     * @param history_buffer Reference to metrics history
     */
    explicit AnalyticsDashboard(const MetricsHistoryBuffer& history_buffer);

    /**
     * @brief Analyze trends for all metrics
     * @return Vector of TrendAnalysis for each metric
     */
    std::vector<TrendAnalysis> analyze_trends();

    /**
     * @brief Analyze correlation between metrics
     * @param metric_threshold Minimum correlation to report (0.5 = 50%)
     * @return Vector of correlated metric pairs
     */
    std::vector<CorrelationPair> analyze_correlations(double metric_threshold = 0.5);

    /**
     * @brief Detect anomalies in all metrics
     * @return Vector of anomaly reports
     */
    std::vector<AnomalyReport> detect_anomalies();

    /**
     * @brief Predict future trends
     * @param prediction_minutes How far ahead to predict (5, 10, 15, 30)
     * @return Vector of prediction reports
     */
    std::vector<PredictionReport> predict_trends(int prediction_minutes = 5);

    /**
     * @brief Generate comprehensive statistical report
     * @param time_window "last_5min", "last_hour", "last_day"
     * @return Complete statistical report
     */
    StatisticalReport generate_statistical_report(const std::string& time_window = "last_5min");

    /**
     * @brief Analyze performance characteristics
     * @return Performance metrics and bottleneck analysis
     */
    PerformanceMetrics analyze_performance();

    /**
     * @brief Export report to JSON format
     */
    nlohmann::json export_report_json(const StatisticalReport& report);

    /**
     * @brief Export report to CSV format
     */
    std::string export_report_csv(const StatisticalReport& report);

    /**
     * @brief Export report to Markdown format (human-readable)
     */
    std::string export_report_markdown(const StatisticalReport& report);

    /**
     * @brief Get correlation matrix visualization data
     * @return 2D array suitable for heatmap rendering
     */
    std::vector<std::vector<double>> get_correlation_matrix();

    /**
     * @brief Get risk assessment for next time period
     * @param minutes_ahead How far to predict
     * @return Risk level (0-100) and recommendations
     */
    struct RiskAssessment {
        double risk_score;  ///< 0-100
        std::string level;  ///< "low", "medium", "high", "critical"
        std::vector<std::string> risk_factors;
        std::vector<std::string> recommendations;
    };
    RiskAssessment assess_risk(int minutes_ahead = 5);

    /**
     * @brief Compare metrics against historical baselines
     * @return Deviation percentages and alerts
     */
    struct BaselineComparison {
        std::string metric_name;
        double current_value;
        double baseline_avg;
        double deviation_percent;
        std::string status;  ///< "normal", "elevated", "critical"
    };
    std::vector<BaselineComparison> compare_to_baseline();

    /**
     * @brief Get optimization suggestions based on analysis
     */
    std::vector<std::string> get_optimization_suggestions();

    /**
     * @brief Generate executive summary for the dashboard
     */
    struct ExecutiveSummary {
        std::string overall_status;
        std::string key_findings;
        std::vector<std::string> top_issues;
        std::vector<std::string> recommendations;
        double uptime_percent;
        int critical_events;
        int warning_events;
    };
    ExecutiveSummary generate_executive_summary();

private:
    const MetricsHistoryBuffer& history_;
    
    // Helper methods
    std::vector<double> extract_metric(
        std::function<double(const MetricsPoint&)> accessor) const;
    
    std::string correlation_to_relationship(double correlation) const;
    
    std::string anomaly_severity_assessment(double anomaly_rate) const;
    
    double calculate_trend_strength(const std::vector<double>& values) const;
    
    int detect_periodicity(const std::vector<double>& values) const;
    
    std::vector<std::string> generate_recommendations(
        const StatisticalReport& report) const;
};

}  // namespace ros2_dashboard
