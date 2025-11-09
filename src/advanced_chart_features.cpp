/**
 * @file advanced_chart_features.cpp
 * @brief Advanced charting features implementation
 */

#include "advanced_chart_features.hpp"
#include <numeric>
#include <cmath>
#include <iostream>

namespace ros2_dashboard::charts {

std::vector<AnomalyInfo> ChartAnalytics::detect_anomalies(
    const std::vector<double>& data, 
    double threshold) {
    
    std::vector<AnomalyInfo> anomalies;
    if (data.size() < 2) return anomalies;

    auto stats = calculate_stats(data);
    
    for (size_t i = 0; i < data.size(); ++i) {
        double z_score = (stats.std_dev > 0) ? 
            (data[i] - stats.mean) / stats.std_dev : 0;
        
        double deviation_percent = (stats.mean > 0) ?
            std::abs((data[i] - stats.mean) / stats.mean) * 100 : 0;
        
        bool is_anomaly = std::abs(z_score) > threshold;
        
        if (is_anomaly || deviation_percent > 50) {
            anomalies.push_back({
                i, 
                data[i],
                z_score,
                deviation_percent,
                is_anomaly
            });
        }
    }
    
    return anomalies;
}

PredictionInfo ChartAnalytics::predict_trend(
    const std::vector<double>& data,
    size_t prediction_steps) {
    
    PredictionInfo info;
    info.predicted_values.clear();
    
    if (data.size() < 2) {
        info.trend = "insufficient_data";
        info.r_squared = 0;
        return info;
    }

    // Linear regression (simple trend line)
    double n = data.size();
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0, sum_y2 = 0;
    
    for (size_t i = 0; i < data.size(); ++i) {
        double x = i;
        double y = data[i];
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
        sum_y2 += y * y;
    }

    double slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
    double intercept = (sum_y - slope * sum_x) / n;

    // Calculate R-squared
    double ss_res = 0, ss_tot = 0;
    double mean_y = sum_y / n;
    
    for (size_t i = 0; i < data.size(); ++i) {
        double predicted = slope * i + intercept;
        ss_res += (data[i] - predicted) * (data[i] - predicted);
        ss_tot += (data[i] - mean_y) * (data[i] - mean_y);
    }

    info.r_squared = (ss_tot > 0) ? 1 - (ss_res / ss_tot) : 0;
    info.slope = slope;

    // Predict next points
    double last_x = data.size();
    for (size_t i = 0; i < prediction_steps; ++i) {
        double predicted = slope * (last_x + i) + intercept;
        info.predicted_values.push_back(predicted);
    }

    // Determine trend
    if (std::abs(slope) < 0.001) {
        info.trend = "stable";
    } else if (slope > 0) {
        info.trend = "increasing";
    } else {
        info.trend = "decreasing";
    }

    return info;
}

ChartAnalytics::Stats ChartAnalytics::calculate_stats(const std::vector<double>& data) {
    Stats stats;
    if (data.empty()) return stats;

    // Mean
    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    stats.mean = sum / data.size();

    // Min/Max
    auto [min_it, max_it] = std::minmax_element(data.begin(), data.end());
    stats.min = *min_it;
    stats.max = *max_it;

    // Standard deviation
    double variance = 0;
    for (auto value : data) {
        variance += (value - stats.mean) * (value - stats.mean);
    }
    variance /= data.size();
    stats.std_dev = std::sqrt(variance);

    // Median
    std::vector<double> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end());
    size_t mid = sorted_data.size() / 2;
    stats.median = (sorted_data.size() % 2 == 0) ?
        (sorted_data[mid - 1] + sorted_data[mid]) / 2 : sorted_data[mid];

    // 95th percentile
    size_t p95_idx = static_cast<size_t>(0.95 * sorted_data.size());
    if (p95_idx < sorted_data.size()) {
        stats.percentile_95 = sorted_data[p95_idx];
    }

    return stats;
}

std::vector<double> ChartAnalytics::smooth_ema(
    const std::vector<double>& data,
    double alpha) {
    
    std::vector<double> smoothed;
    if (data.empty()) return smoothed;

    smoothed.push_back(data[0]);
    for (size_t i = 1; i < data.size(); ++i) {
        double ema = alpha * data[i] + (1 - alpha) * smoothed[i - 1];
        smoothed.push_back(ema);
    }
    
    return smoothed;
}

std::vector<double> ChartAnalytics::smooth_moving_average(
    const std::vector<double>& data,
    size_t window_size) {
    
    std::vector<double> smoothed;
    if (data.size() < window_size) return data;

    for (size_t i = 0; i < data.size() - window_size + 1; ++i) {
        double sum = 0;
        for (size_t j = 0; j < window_size; ++j) {
            sum += data[i + j];
        }
        smoothed.push_back(sum / window_size);
    }
    
    return smoothed;
}

double ChartAnalytics::calculate_correlation(
    const std::vector<double>& data1,
    const std::vector<double>& data2) {
    
    if (data1.size() != data2.size() || data1.empty()) return 0;

    double n = data1.size();
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0, sum_y2 = 0;

    for (size_t i = 0; i < n; ++i) {
        sum_x += data1[i];
        sum_y += data2[i];
        sum_xy += data1[i] * data2[i];
        sum_x2 += data1[i] * data1[i];
        sum_y2 += data2[i] * data2[i];
    }

    double numerator = n * sum_xy - sum_x * sum_y;
    double denominator = std::sqrt((n * sum_x2 - sum_x * sum_x) * 
                                    (n * sum_y2 - sum_y * sum_y));

    return (denominator > 0) ? numerator / denominator : 0;
}

std::vector<std::vector<double>> ChartAnalytics::correlation_matrix(
    const std::vector<std::vector<double>>& datasets) {
    
    std::vector<std::vector<double>> matrix(datasets.size(), 
                                           std::vector<double>(datasets.size(), 0));

    for (size_t i = 0; i < datasets.size(); ++i) {
        for (size_t j = 0; j < datasets.size(); ++j) {
            if (i == j) {
                matrix[i][j] = 1.0;
            } else {
                matrix[i][j] = calculate_correlation(datasets[i], datasets[j]);
            }
        }
    }

    return matrix;
}

// ChartNavigator implementation
ChartNavigator::ChartNavigator(size_t total_points)
    : total_points_(total_points),
      start_index_(0),
      end_index_(total_points > 0 ? total_points - 1 : 0),
      zoom_level_(1.0) {
}

void ChartNavigator::zoom_in(size_t center_index, double zoom_factor) {
    size_t visible_range = (end_index_ - start_index_ + 1) / zoom_factor;
    if (visible_range < 1) visible_range = 1;

    size_t new_start = (center_index > visible_range / 2) ? 
        center_index - visible_range / 2 : 0;
    size_t new_end = new_start + visible_range - 1;

    if (new_end >= total_points_) {
        new_end = total_points_ - 1;
        new_start = (new_end > visible_range) ? new_end - visible_range + 1 : 0;
    }

    start_index_ = new_start;
    end_index_ = new_end;
    zoom_level_ *= zoom_factor;
}

void ChartNavigator::zoom_out() {
    if (zoom_level_ > 1.0) {
        zoom_level_ /= 1.5;
        if (zoom_level_ < 1.0) zoom_level_ = 1.0;
        
        size_t visible_range = (total_points_ * zoom_level_);
        size_t center = (start_index_ + end_index_) / 2;
        
        size_t new_start = (center > visible_range / 2) ?
            center - visible_range / 2 : 0;
        size_t new_end = new_start + visible_range - 1;

        if (new_end >= total_points_) {
            new_end = total_points_ - 1;
            new_start = (new_end > visible_range) ? new_end - visible_range + 1 : 0;
        }

        start_index_ = new_start;
        end_index_ = new_end;
    }
}

void ChartNavigator::pan(int delta) {
    int new_start = static_cast<int>(start_index_) + delta;
    int new_end = static_cast<int>(end_index_) + delta;

    if (new_start < 0) {
        new_start = 0;
        new_end = end_index_ - start_index_;
    } else if (new_end >= static_cast<int>(total_points_)) {
        new_end = total_points_ - 1;
        new_start = new_end - (end_index_ - start_index_);
    }

    start_index_ = new_start;
    end_index_ = new_end;
}

void ChartNavigator::set_range(size_t start_index, size_t end_index) {
    if (start_index < end_index && end_index < total_points_) {
        start_index_ = start_index;
        end_index_ = end_index;
        zoom_level_ = static_cast<double>(total_points_) / 
                     (end_index - start_index + 1);
    }
}

ChartNavigator::Range ChartNavigator::get_current_range() const {
    return {start_index_, end_index_, zoom_level_};
}

void ChartNavigator::reset() {
    start_index_ = 0;
    end_index_ = (total_points_ > 0) ? total_points_ - 1 : 0;
    zoom_level_ = 1.0;
}

}  // namespace ros2_dashboard::charts
