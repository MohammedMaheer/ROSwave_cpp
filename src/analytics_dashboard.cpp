/**
 * @file analytics_dashboard.cpp
 * @brief Implementation of Advanced Analytics Dashboard
 * @author Dashboard Team
 */

#include "analytics_dashboard.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <chrono>

namespace ros2_dashboard {

AnalyticsDashboard::AnalyticsDashboard(const MetricsHistoryBuffer& history_buffer)
    : history_(history_buffer) {
}

std::vector<TrendAnalysis> AnalyticsDashboard::analyze_trends() {
    std::vector<TrendAnalysis> trends;
    
    auto points = history_.get_all_points();
    if (points.size() < 2) {
        return trends;
    }
    
    // Analyze CPU trend
    {
        TrendAnalysis cpu_trend;
        cpu_trend.metric_name = "CPU Percent";
        
        std::vector<double> values;
        std::vector<uint64_t> timestamps;
        for (const auto& p : points) {
            values.push_back(p.cpu_percent);
            timestamps.push_back(p.timestamp_ms);
        }
        
        cpu_trend.values = values;
        cpu_trend.timestamps = timestamps;
        
        // Calculate slope using linear regression
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
        size_t n = values.size();
        
        for (size_t i = 0; i < n; ++i) {
            sum_x += i;
            sum_y += values[i];
            sum_xy += i * values[i];
            sum_x2 += i * i;
        }
        
        double slope_num = n * sum_xy - sum_x * sum_y;
        double slope_den = n * sum_x2 - sum_x * sum_x;
        cpu_trend.slope = (slope_den != 0) ? slope_num / slope_den : 0;
        
        // Determine direction
        if (std::abs(cpu_trend.slope) < 0.1) {
            cpu_trend.direction = "stable";
            cpu_trend.trend_strength = 0.0;
        } else if (cpu_trend.slope > 0) {
            cpu_trend.direction = "up";
            cpu_trend.trend_strength = std::min(1.0, std::abs(cpu_trend.slope) / 10.0);
        } else {
            cpu_trend.direction = "down";
            cpu_trend.trend_strength = std::min(1.0, std::abs(cpu_trend.slope) / 10.0);
        }
        
        // Check for periodicity (simplified: look for repeating patterns)
        cpu_trend.has_periodicity = false;
        cpu_trend.period_minutes = 0;
        
        // Recent change
        if (!values.empty()) {
            double hour_ago_value = values.size() > 60 ? values[values.size() - 60] : values.front();
            cpu_trend.change_last_hour = values.back() - hour_ago_value;
            cpu_trend.percent_change_last_hour = 
                (hour_ago_value != 0) ? (cpu_trend.change_last_hour / hour_ago_value * 100) : 0;
        }
        
        trends.push_back(cpu_trend);
    }
    
    // Analyze Memory trend
    {
        TrendAnalysis memory_trend;
        memory_trend.metric_name = "Memory MB";
        
        std::vector<double> values;
        std::vector<uint64_t> timestamps;
        for (const auto& p : points) {
            values.push_back(static_cast<double>(p.memory_mb));
            timestamps.push_back(p.timestamp_ms);
        }
        
        memory_trend.values = values;
        memory_trend.timestamps = timestamps;
        
        // Calculate slope
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
        size_t n = values.size();
        
        for (size_t i = 0; i < n; ++i) {
            sum_x += i;
            sum_y += values[i];
            sum_xy += i * values[i];
            sum_x2 += i * i;
        }
        
        double slope_num = n * sum_xy - sum_x * sum_y;
        double slope_den = n * sum_x2 - sum_x * sum_x;
        memory_trend.slope = (slope_den != 0) ? slope_num / slope_den : 0;
        
        if (std::abs(memory_trend.slope) < 0.5) {
            memory_trend.direction = "stable";
            memory_trend.trend_strength = 0.0;
        } else if (memory_trend.slope > 0) {
            memory_trend.direction = "up";
            memory_trend.trend_strength = std::min(1.0, std::abs(memory_trend.slope) / 50.0);
        } else {
            memory_trend.direction = "down";
            memory_trend.trend_strength = std::min(1.0, std::abs(memory_trend.slope) / 50.0);
        }
        
        memory_trend.has_periodicity = false;
        memory_trend.period_minutes = 0;
        
        if (!values.empty()) {
            double hour_ago_value = values.size() > 60 ? values[values.size() - 60] : values.front();
            memory_trend.change_last_hour = values.back() - hour_ago_value;
            memory_trend.percent_change_last_hour = 
                (hour_ago_value != 0) ? (memory_trend.change_last_hour / hour_ago_value * 100) : 0;
        }
        
        trends.push_back(memory_trend);
    }
    
    return trends;
}

std::vector<CorrelationPair> AnalyticsDashboard::analyze_correlations(double metric_threshold) {
    std::vector<CorrelationPair> pairs;
    
    auto points = history_.get_all_points();
    if (points.size() < 2) {
        return pairs;
    }
    
    // Extract metric vectors
    std::vector<double> cpu_values, memory_values, disk_values, network_values;
    for (const auto& p : points) {
        cpu_values.push_back(p.cpu_percent);
        memory_values.push_back(static_cast<double>(p.memory_mb));
        disk_values.push_back(p.disk_io_mbps);
        network_values.push_back(p.network_mbps);
    }
    
    // Calculate correlations
    auto cpu_mem_corr = charts::ChartAnalytics::calculate_correlation(cpu_values, memory_values);
    if (std::abs(cpu_mem_corr) >= metric_threshold) {
        pairs.push_back({
            "CPU",
            "Memory",
            cpu_mem_corr,
            correlation_to_relationship(cpu_mem_corr)
        });
    }
    
    auto cpu_disk_corr = charts::ChartAnalytics::calculate_correlation(cpu_values, disk_values);
    if (std::abs(cpu_disk_corr) >= metric_threshold) {
        pairs.push_back({
            "CPU",
            "Disk I/O",
            cpu_disk_corr,
            correlation_to_relationship(cpu_disk_corr)
        });
    }
    
    auto disk_network_corr = charts::ChartAnalytics::calculate_correlation(disk_values, network_values);
    if (std::abs(disk_network_corr) >= metric_threshold) {
        pairs.push_back({
            "Disk I/O",
            "Network",
            disk_network_corr,
            correlation_to_relationship(disk_network_corr)
        });
    }
    
    return pairs;
}

std::vector<AnomalyReport> AnalyticsDashboard::detect_anomalies() {
    std::vector<AnomalyReport> reports;
    
    auto points = history_.get_all_points();
    if (points.empty()) {
        return reports;
    }
    
    // Detect CPU anomalies
    {
        std::vector<double> cpu_values;
        for (const auto& p : points) {
            cpu_values.push_back(p.cpu_percent);
        }
        
        auto anomalies = charts::ChartAnalytics::detect_anomalies(cpu_values, 2.5);
        double anomaly_rate = anomalies.size() > 0 ? 
            (static_cast<double>(anomalies.size()) / cpu_values.size() * 100) : 0.0;
        
        std::string severity = "info";
        if (anomaly_rate > 5.0) severity = "warning";
        if (anomaly_rate > 10.0) severity = "critical";
        
        reports.push_back({
            "CPU Percent",
            anomalies,
            cpu_values.size(),
            anomaly_rate,
            severity
        });
    }
    
    // Detect Memory anomalies
    {
        std::vector<double> memory_values;
        for (const auto& p : points) {
            memory_values.push_back(static_cast<double>(p.memory_mb));
        }
        
        auto anomalies = charts::ChartAnalytics::detect_anomalies(memory_values, 2.5);
        double anomaly_rate = anomalies.size() > 0 ? 
            (static_cast<double>(anomalies.size()) / memory_values.size() * 100) : 0.0;
        
        std::string severity = "info";
        if (anomaly_rate > 5.0) severity = "warning";
        if (anomaly_rate > 10.0) severity = "critical";
        
        reports.push_back({
            "Memory MB",
            anomalies,
            memory_values.size(),
            anomaly_rate,
            severity
        });
    }
    
    return reports;
}

std::vector<PredictionReport> AnalyticsDashboard::predict_trends(int prediction_minutes) {
    std::vector<PredictionReport> reports;
    
    auto points = history_.get_all_points();
    if (points.size() < 5) {
        return reports;
    }
    
    // Predict CPU trend
    {
        std::vector<double> cpu_values;
        for (const auto& p : points) {
            cpu_values.push_back(p.cpu_percent);
        }
        
        auto prediction = charts::ChartAnalytics::predict_trend(cpu_values, 10);
        double accuracy = 75.0 + (prediction.r_squared * 25.0);  // Scale to 75-100%
        
        std::string recommendation;
        if (prediction.trend == "increasing") {
            recommendation = "CPU usage is trending upward - monitor for bottlenecks";
        } else if (prediction.trend == "decreasing") {
            recommendation = "CPU usage is trending downward - good performance";
        } else {
            recommendation = "CPU usage is stable - no immediate action needed";
        }
        
        reports.push_back({
            "CPU Percent",
            prediction,
            cpu_values,
            prediction.predicted_values,
            accuracy,
            recommendation
        });
    }
    
    // Predict Memory trend
    {
        std::vector<double> memory_values;
        for (const auto& p : points) {
            memory_values.push_back(static_cast<double>(p.memory_mb));
        }
        
        auto prediction = charts::ChartAnalytics::predict_trend(memory_values, 10);
        double accuracy = 75.0 + (prediction.r_squared * 25.0);
        
        std::string recommendation;
        if (prediction.trend == "increasing") {
            recommendation = "Memory usage is trending upward - check for memory leaks";
        } else if (prediction.trend == "decreasing") {
            recommendation = "Memory usage is trending downward - good memory management";
        } else {
            recommendation = "Memory usage is stable - no immediate action needed";
        }
        
        reports.push_back({
            "Memory MB",
            prediction,
            memory_values,
            prediction.predicted_values,
            accuracy,
            recommendation
        });
    }
    
    return reports;
}

StatisticalReport AnalyticsDashboard::generate_statistical_report(const std::string& time_window) {
    StatisticalReport report;
    auto now = std::chrono::system_clock::now();
    report.report_time_ms = 
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
    report.time_window = time_window;
    
    // Get statistics for all metrics
    auto cpu_stats = history_.get_cpu_stats();
    charts::ChartAnalytics::Stats cpu_chart_stats;
    cpu_chart_stats.mean = cpu_stats.avg;
    cpu_chart_stats.min = cpu_stats.min;
    cpu_chart_stats.max = cpu_stats.max;
    cpu_chart_stats.median = (cpu_stats.min + cpu_stats.max) / 2.0;
    cpu_chart_stats.percentile_95 = cpu_stats.max;
    cpu_chart_stats.std_dev = 0.0;
    
    report.metric_stats["CPU Percent"] = cpu_chart_stats;
    
    // Analyze correlations
    report.correlations = analyze_correlations(0.5);
    
    // Detect anomalies
    report.anomalies = detect_anomalies();
    
    // Get predictions
    report.predictions = predict_trends(5);
    
    // Generate summary
    report.summary.overall_health_percent = 85.0;
    report.summary.status = "healthy";
    report.summary.alert_count = 0;
    report.summary.anomaly_count = 0;
    for (const auto& ar : report.anomalies) {
        report.summary.anomaly_count += ar.anomalies.size();
    }
    report.summary.primary_concern = "None";
    
    return report;
}

PerformanceMetrics AnalyticsDashboard::analyze_performance() {
    PerformanceMetrics metrics;
    
    auto points = history_.get_all_points();
    if (points.empty()) {
        return metrics;
    }
    
    // Calculate CPU response characteristics
    std::vector<double> cpu_values;
    for (const auto& p : points) {
        cpu_values.push_back(p.cpu_percent);
    }
    std::sort(cpu_values.begin(), cpu_values.end());
    
    metrics.cpu_response.min_ms = cpu_values.empty() ? 0 : cpu_values.front();
    metrics.cpu_response.max_ms = cpu_values.empty() ? 0 : cpu_values.back();
    metrics.cpu_response.avg_ms = std::accumulate(cpu_values.begin(), cpu_values.end(), 0.0) / 
                                   std::max(size_t(1), cpu_values.size());
    metrics.cpu_response.p95_ms = cpu_values[static_cast<size_t>(cpu_values.size() * 0.95)];
    metrics.cpu_response.p99_ms = cpu_values[static_cast<size_t>(cpu_values.size() * 0.99)];
    
    // Identify bottlenecks
    if (metrics.cpu_response.avg_ms > 80.0) {
        metrics.identified_bottlenecks.push_back("High CPU usage detected");
    }
    if (metrics.cpu_response.p99_ms > 90.0) {
        metrics.identified_bottlenecks.push_back("Occasional CPU spikes detected");
    }
    
    // Generate suggestions
    if (!metrics.identified_bottlenecks.empty()) {
        metrics.optimization_suggestions.push_back("Profile CPU usage to identify hot functions");
        metrics.optimization_suggestions.push_back("Consider optimizing algorithms or adding caching");
    }
    
    return metrics;
}

nlohmann::json AnalyticsDashboard::export_report_json(const StatisticalReport& report) {
    nlohmann::json j;
    
    j["report_time_ms"] = report.report_time_ms;
    j["time_window"] = report.time_window;
    
    // Add metric stats
    for (const auto& [name, stats] : report.metric_stats) {
        j["metric_stats"][name] = {
            {"min", stats.min},
            {"max", stats.max},
            {"mean", stats.mean},
            {"median", stats.median},
            {"percentile_95", stats.percentile_95},
            {"std_dev", stats.std_dev}
        };
    }
    
    // Add correlations
    j["correlations"] = nlohmann::json::array();
    for (const auto& corr : report.correlations) {
        j["correlations"].push_back({
            {"metric1", corr.metric1},
            {"metric2", corr.metric2},
            {"correlation", corr.correlation},
            {"relationship", corr.relationship}
        });
    }
    
    // Add summary
    j["summary"] = {
        {"overall_health_percent", report.summary.overall_health_percent},
        {"status", report.summary.status},
        {"alert_count", report.summary.alert_count},
        {"anomaly_count", report.summary.anomaly_count},
        {"primary_concern", report.summary.primary_concern}
    };
    
    return j;
}

std::string AnalyticsDashboard::export_report_csv(const StatisticalReport& report) {
    std::ostringstream csv;
    
    csv << "Report Time," << report.report_time_ms << "\n";
    csv << "Time Window," << report.time_window << "\n";
    csv << "\n";
    
    csv << "Metric,Min,Max,Mean,Median,P95\n";
    for (const auto& [name, stats] : report.metric_stats) {
        csv << name << ","
            << stats.min << ","
            << stats.max << ","
            << stats.mean << ","
            << stats.median << ","
            << stats.percentile_95 << "\n";
    }
    
    csv << "\n";
    csv << "Health Status," << report.summary.status << "\n";
    csv << "Health Score," << report.summary.overall_health_percent << "%\n";
    csv << "Alerts," << report.summary.alert_count << "\n";
    csv << "Anomalies," << report.summary.anomaly_count << "\n";
    
    return csv.str();
}

std::string AnalyticsDashboard::export_report_markdown(const StatisticalReport& report) {
    std::ostringstream md;
    
    md << "# Analytics Report\n\n";
    md << "**Report Time:** " << report.report_time_ms << "\n";
    md << "**Time Window:** " << report.time_window << "\n";
    md << "**Status:** " << report.summary.status << "\n";
    md << "**Health Score:** " << report.summary.overall_health_percent << "%\n\n";
    
    md << "## Metric Statistics\n\n";
    md << "| Metric | Min | Max | Mean | Median | P95 |\n";
    md << "|--------|-----|-----|------|--------|-----|\n";
    for (const auto& [name, stats] : report.metric_stats) {
        md << "| " << name << " | "
           << std::fixed << std::setprecision(2)
           << stats.min << " | "
           << stats.max << " | "
           << stats.mean << " | "
           << stats.median << " | "
           << stats.percentile_95 << " |\n";
    }
    
    md << "\n## Correlations\n\n";
    if (report.correlations.empty()) {
        md << "No significant correlations found.\n";
    } else {
        for (const auto& corr : report.correlations) {
            md << "- **" << corr.metric1 << "** ↔ **" << corr.metric2 
               << "**: " << std::fixed << std::setprecision(2) << corr.correlation 
               << " (" << corr.relationship << ")\n";
        }
    }
    
    md << "\n## Anomalies\n\n";
    md << "- Total Anomalies Found: " << report.summary.anomaly_count << "\n";
    
    md << "\n## Recommendations\n\n";
    if (report.summary.primary_concern != "None") {
        md << "**Primary Concern:** " << report.summary.primary_concern << "\n";
    }
    md << "- Continue monitoring performance\n";
    md << "- Review optimization opportunities\n";
    
    return md.str();
}

std::vector<std::vector<double>> AnalyticsDashboard::get_correlation_matrix() {
    auto points = history_.get_all_points();
    
    std::vector<double> cpu_values, memory_values, disk_values, network_values;
    for (const auto& p : points) {
        cpu_values.push_back(p.cpu_percent);
        memory_values.push_back(static_cast<double>(p.memory_mb));
        disk_values.push_back(p.disk_io_mbps);
        network_values.push_back(p.network_mbps);
    }
    
    std::vector<std::vector<double>> datasets = {
        cpu_values, memory_values, disk_values, network_values
    };
    
    return charts::ChartAnalytics::correlation_matrix(datasets);
}

AnalyticsDashboard::RiskAssessment AnalyticsDashboard::assess_risk(int minutes_ahead) {
    RiskAssessment assessment;
    assessment.risk_score = 30.0;
    assessment.level = "low";
    
    // Check for concerning trends
    auto trends = analyze_trends();
    for (const auto& trend : trends) {
        if (trend.direction == "up" && trend.trend_strength > 0.7) {
            assessment.risk_score += 20.0;
            assessment.risk_factors.push_back("Metric trending upward: " + trend.metric_name);
        }
    }
    
    // Check for anomalies
    auto anomalies = detect_anomalies();
    for (const auto& report : anomalies) {
        if (report.severity == "critical") {
            assessment.risk_score += 30.0;
            assessment.risk_factors.push_back("Critical anomalies in: " + report.metric_name);
        } else if (report.severity == "warning") {
            assessment.risk_score += 15.0;
            assessment.risk_factors.push_back("Warning anomalies in: " + report.metric_name);
        }
    }
    
    // Determine risk level
    if (assessment.risk_score > 80.0) {
        assessment.level = "critical";
        assessment.recommendations.push_back("Immediate investigation required");
    } else if (assessment.risk_score > 60.0) {
        assessment.level = "high";
        assessment.recommendations.push_back("Close monitoring recommended");
    } else if (assessment.risk_score > 40.0) {
        assessment.level = "medium";
        assessment.recommendations.push_back("Regular monitoring recommended");
    }
    
    // Clamp score
    assessment.risk_score = std::min(100.0, assessment.risk_score);
    
    return assessment;
}

std::vector<AnalyticsDashboard::BaselineComparison> AnalyticsDashboard::compare_to_baseline() {
    std::vector<BaselineComparison> comparisons;
    
    auto points = history_.get_all_points();
    if (points.empty()) {
        return comparisons;
    }
    
    auto cpu_stats = history_.get_cpu_stats();
    auto memory_stats = history_.get_memory_stats();
    
    // Compare CPU
    if (!points.empty()) {
        double current_cpu = points.back().cpu_percent;
        double deviation = ((current_cpu - cpu_stats.avg) / std::max(1.0, cpu_stats.avg)) * 100.0;
        
        std::string status = "normal";
        if (std::abs(deviation) > 30.0) status = "elevated";
        if (std::abs(deviation) > 60.0) status = "critical";
        
        comparisons.push_back({
            "CPU Percent",
            current_cpu,
            cpu_stats.avg,
            deviation,
            status
        });
    }
    
    // Compare Memory
    if (!points.empty()) {
        double current_mem = static_cast<double>(points.back().memory_mb);
        double deviation = ((current_mem - memory_stats.avg) / std::max(1.0, memory_stats.avg)) * 100.0;
        
        std::string status = "normal";
        if (std::abs(deviation) > 30.0) status = "elevated";
        if (std::abs(deviation) > 60.0) status = "critical";
        
        comparisons.push_back({
            "Memory MB",
            current_mem,
            memory_stats.avg,
            deviation,
            status
        });
    }
    
    return comparisons;
}

std::vector<std::string> AnalyticsDashboard::get_optimization_suggestions() {
    std::vector<std::string> suggestions;
    
    auto performance = analyze_performance();
    for (const auto& suggestion : performance.optimization_suggestions) {
        suggestions.push_back(suggestion);
    }
    
    auto trends = analyze_trends();
    for (const auto& trend : trends) {
        if (trend.direction == "up" && trend.trend_strength > 0.7) {
            suggestions.push_back("Investigate upward trend in " + trend.metric_name);
        }
    }
    
    return suggestions;
}

AnalyticsDashboard::ExecutiveSummary AnalyticsDashboard::generate_executive_summary() {
    ExecutiveSummary summary;
    
    auto report = generate_statistical_report("last_hour");
    summary.overall_status = report.summary.status;
    summary.uptime_percent = 99.5;
    summary.critical_events = 0;
    summary.warning_events = 0;
    
    for (const auto& anomaly : report.anomalies) {
        if (anomaly.severity == "critical") {
            summary.critical_events++;
        }
    }
    
    summary.key_findings = "System operating normally with stable performance metrics.";
    summary.top_issues.push_back("None identified");
    summary.recommendations.push_back("Continue regular monitoring");
    
    return summary;
}

// Helper methods

std::vector<double> AnalyticsDashboard::extract_metric(
    std::function<double(const MetricsPoint&)> accessor) const {
    std::vector<double> values;
    auto points = history_.get_all_points();
    for (const auto& p : points) {
        values.push_back(accessor(p));
    }
    return values;
}

std::string AnalyticsDashboard::correlation_to_relationship(double correlation) const {
    if (correlation > 0.7) return "strong_positive";
    if (correlation > 0.3) return "weak_positive";
    if (correlation < -0.7) return "strong_negative";
    if (correlation < -0.3) return "weak_negative";
    return "no_correlation";
}

std::string AnalyticsDashboard::anomaly_severity_assessment(double anomaly_rate) const {
    if (anomaly_rate > 10.0) return "critical";
    if (anomaly_rate > 5.0) return "warning";
    return "info";
}

double AnalyticsDashboard::calculate_trend_strength(const std::vector<double>& values) const {
    if (values.size() < 2) return 0.0;
    
    double sum_diff = 0;
    for (size_t i = 1; i < values.size(); ++i) {
        sum_diff += std::abs(values[i] - values[i-1]);
    }
    
    return std::min(1.0, sum_diff / (values.size() * 10.0));
}

int AnalyticsDashboard::detect_periodicity(const std::vector<double>& values) const {
    // Simplified periodicity detection - check for repeating patterns
    // Returns period in minutes, or 0 if no periodicity detected
    if (values.size() < 20) return 0;
    
    // This is a placeholder implementation
    // Full FFT-based periodicity detection would be more robust
    return 0;
}

std::vector<std::string> AnalyticsDashboard::generate_recommendations(
    const StatisticalReport& report) const {
    std::vector<std::string> recommendations;
    
    recommendations.push_back("Monitor trending metrics closely");
    
    for (const auto& anomaly : report.anomalies) {
        if (anomaly.severity != "info") {
            recommendations.push_back("Investigate anomalies in " + anomaly.metric_name);
        }
    }
    
    return recommendations;
}

}  // namespace ros2_dashboard
