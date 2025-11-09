/**
 * @file advanced_chart_features.hpp
 * @brief Advanced charting features (zoom, pan, anomaly detection, prediction)
 * @author Dashboard Team
 */

#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <cmath>
#include <algorithm>

namespace ros2_dashboard::charts {

/**
 * @struct AnomalyInfo
 * @brief Information about detected anomalies
 */
struct AnomalyInfo {
    size_t index;              // Position in data
    double value;              // The anomalous value
    double z_score;            // Statistical Z-score
    double deviation_percent;  // % deviation from mean
    bool is_anomaly;
};

/**
 * @struct PredictionInfo
 * @brief Trend prediction for next N seconds
 */
struct PredictionInfo {
    std::vector<double> predicted_values;
    double slope;              // Trend slope
    double r_squared;          // Prediction confidence (0-1)
    std::string trend;         // "increasing", "decreasing", "stable"
};

/**
 * @class ChartAnalytics
 * @brief Statistical analysis and predictions for chart data
 */
class ChartAnalytics {
public:
    /**
     * @brief Detect anomalies using Z-score method
     * @param data Historical data points
     * @param threshold Z-score threshold (default 2.5 = 2.5 std devs)
     * @return List of detected anomalies
     */
    static std::vector<AnomalyInfo> detect_anomalies(
        const std::vector<double>& data, 
        double threshold = 2.5);

    /**
     * @brief Predict future trend
     * @param data Historical data points
     * @param prediction_steps How many steps to predict ahead
     * @return Prediction information
     */
    static PredictionInfo predict_trend(
        const std::vector<double>& data,
        size_t prediction_steps = 10);

    /**
     * @brief Calculate statistics
     */
    struct Stats {
        double mean = 0;
        double std_dev = 0;
        double min = 0;
        double max = 0;
        double median = 0;
        double percentile_95 = 0;
    };
    
    static Stats calculate_stats(const std::vector<double>& data);

    /**
     * @brief Smooth data using exponential moving average
     */
    static std::vector<double> smooth_ema(
        const std::vector<double>& data,
        double alpha = 0.3);

    /**
     * @brief Smooth data using moving average
     */
    static std::vector<double> smooth_moving_average(
        const std::vector<double>& data,
        size_t window_size = 5);

    /**
     * @brief Calculate correlation between two datasets
     */
    static double calculate_correlation(
        const std::vector<double>& data1,
        const std::vector<double>& data2);

    /**
     * @brief Generate heatmap-style correlation matrix
     */
    static std::vector<std::vector<double>> correlation_matrix(
        const std::vector<std::vector<double>>& datasets);
};

/**
 * @class ChartNavigator
 * @brief Handles zoom, pan, and range selection for charts
 */
class ChartNavigator {
public:
    explicit ChartNavigator(size_t total_points);

    /**
     * @brief Zoom in to a range
     * @param center_index Center point for zoom
     * @param zoom_factor Zoom factor (2.0 = 2x zoom)
     */
    void zoom_in(size_t center_index, double zoom_factor = 2.0);

    /**
     * @brief Zoom out
     */
    void zoom_out();

    /**
     * @brief Pan left/right
     * @param delta Number of points to move (negative = left, positive = right)
     */
    void pan(int delta);

    /**
     * @brief Set visible range
     */
    void set_range(size_t start_index, size_t end_index);

    /**
     * @brief Get current visible range
     */
    struct Range {
        size_t start_index;
        size_t end_index;
        size_t visible_points() const { return end_index - start_index + 1; }
        double zoom_level;  // 1.0 = full view, 2.0 = 2x zoom
    };
    Range get_current_range() const;

    /**
     * @brief Reset to full view
     */
    void reset();

private:
    size_t total_points_;
    size_t start_index_;
    size_t end_index_;
    double zoom_level_;
};

}  // namespace ros2_dashboard::charts
