#include "../include/gui/advanced_chart_widget.hpp"
#include <QWheelEvent>
#include <QMouseEvent>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <fstream>

AdvancedTimeSeriesChart::AdvancedTimeSeriesChart(QWidget* parent)
    : QCustomPlot(parent), next_series_id_(0), time_range_(TimeRange::ONE_HOUR),
      forecast_seconds_(60), legend_toggle_enabled_(true), interactive_enabled_(true),
      is_panning_(false) {
    
    setup_axes();
    setInteraction(QCP::iRangeDrag, interactive_enabled_);
    setInteraction(QCP::iRangeZoom, interactive_enabled_);
    setInteraction(QCP::iSelectPlottables, interactive_enabled_);
}

void AdvancedTimeSeriesChart::setup_axes() {
    xAxis->setLabel("Time (s)");
    yAxis->setLabel("Value");
    
    legend->setVisible(true);
    legend->setBrush(QBrush(QColor(255, 255, 255, 200)));
    
    xAxis->grid()->setVisible(true);
    yAxis->grid()->setVisible(true);
}

int AdvancedTimeSeriesChart::add_series(const QString& series_name, const QColor& color) {
    int series_id = next_series_id_++;
    
    SeriesData series;
    series.id = series_id;
    series.name = series_name;
    series.graph = addGraph();
    series.graph->setName(series_name);
    series.graph->setPen(QPen(color, 2));
    series.graph->setLineStyle(QCPGraph::lsLine);
    series.graph->setAdaptiveSampling(true);
    
    series_map_[series_id] = series;
    
    return series_id;
}

void AdvancedTimeSeriesChart::add_data_point(int series_id, double timestamp, double value) {
    auto it = series_map_.find(series_id);
    if (it == series_map_.end()) return;
    
    SeriesData& series = it.value();
    series.times.append(timestamp);
    series.values.append(value);
    series.graph->addData(timestamp, value);
    
    update_series_display(series_id);
}

void AdvancedTimeSeriesChart::clear_series(int series_id) {
    auto it = series_map_.find(series_id);
    if (it == series_map_.end()) return;
    
    it.value().graph->data()->clear();
    it.value().times.clear();
    it.value().values.clear();
}

void AdvancedTimeSeriesChart::remove_series(int series_id) {
    auto it = series_map_.find(series_id);
    if (it != series_map_.end()) {
        removeGraph(it.value().graph);
        series_map_.erase(it);
    }
}

void AdvancedTimeSeriesChart::toggle_series_visibility(int series_id) {
    auto it = series_map_.find(series_id);
    if (it != series_map_.end()) {
        it.value().graph->setVisible(!it.value().graph->visible());
    }
}

AdvancedTimeSeriesChart::SeriesStats AdvancedTimeSeriesChart::get_series_stats(int series_id) const {
    SeriesStats stats;
    
    auto it = series_map_.find(series_id);
    if (it == series_map_.end()) return stats;
    
    const QVector<double>& values = it.value().values;
    if (values.isEmpty()) return stats;
    
    stats.latest = values.back();
    stats.min = *std::min_element(values.begin(), values.end());
    stats.max = *std::max_element(values.begin(), values.end());
    stats.avg = calc_mean(values);
    stats.point_count = values.size();
    
    return stats;
}

void AdvancedTimeSeriesChart::set_show_trend_line(int series_id, bool enabled) {
    auto it = series_map_.find(series_id);
    if (it != series_map_.end()) {
        it.value().show_trend = enabled;
        update_series_display(series_id);
    }
}

void AdvancedTimeSeriesChart::set_show_forecast(int series_id, bool enabled) {
    auto it = series_map_.find(series_id);
    if (it != series_map_.end()) {
        it.value().show_forecast = enabled;
        update_series_display(series_id);
    }
}

void AdvancedTimeSeriesChart::set_forecast_horizon(int seconds) {
    forecast_seconds_ = seconds;
}

void AdvancedTimeSeriesChart::set_anomaly_detection(int series_id, bool enabled, double threshold) {
    auto it = series_map_.find(series_id);
    if (it != series_map_.end()) {
        it.value().anomaly_enabled = enabled;
        it.value().anomaly_threshold = threshold;
    }
}

void AdvancedTimeSeriesChart::set_show_peaks(int series_id, bool enabled) {
    auto it = series_map_.find(series_id);
    if (it != series_map_.end()) {
        it.value().show_peaks = enabled;
    }
}

void AdvancedTimeSeriesChart::set_peak_sensitivity(int series_id, double sensitivity) {
    auto it = series_map_.find(series_id);
    if (it != series_map_.end()) {
        it.value().peak_sensitivity = qBound(0.0, sensitivity, 1.0);
    }
}

void AdvancedTimeSeriesChart::set_time_range(TimeRange range) {
    time_range_ = range;
    emit time_range_changed(range);
}

void AdvancedTimeSeriesChart::set_legend_toggling_enabled(bool enabled) {
    legend_toggle_enabled_ = enabled;
}

void AdvancedTimeSeriesChart::set_interactive_enabled(bool enabled) {
    interactive_enabled_ = enabled;
    setInteraction(QCP::iRangeDrag, enabled);
    setInteraction(QCP::iRangeZoom, enabled);
}

void AdvancedTimeSeriesChart::fit_to_data() {
    rescaleAxes();
}

void AdvancedTimeSeriesChart::refresh() {
    replot();
}

bool AdvancedTimeSeriesChart::export_as_image(const QString& file_path, int width, int height) {
    return savePng(file_path, width, height);
}

bool AdvancedTimeSeriesChart::export_as_csv(const QString& file_path) const {
    std::ofstream file(file_path.toStdString());
    if (!file.is_open()) return false;
    
    // Header
    file << "time";
    for (auto it = series_map_.begin(); it != series_map_.end(); ++it) {
        file << "," << it.value().name.toStdString();
    }
    file << "\n";
    
    // Find max points
    int max_points = 0;
    for (const auto& series : series_map_) {
        max_points = std::max(max_points, series.times.size());
    }
    
    // Data rows
    for (int i = 0; i < max_points; ++i) {
        bool first = true;
        for (auto it = series_map_.begin(); it != series_map_.end(); ++it) {
            if (!first) file << ",";
            if (i < it.value().times.size()) {
                if (first) file << it.value().times[i];
                file << it.value().values[i];
            }
            first = false;
        }
        file << "\n";
    }
    
    file.close();
    return true;
}

QPair<double, double> AdvancedTimeSeriesChart::get_visible_time_range() const {
    return {xAxis->range().lower, xAxis->range().upper};
}

void AdvancedTimeSeriesChart::wheelEvent(QWheelEvent* event) {
    if (!interactive_enabled_) return;
    
    double factor = (event->angleDelta().y() > 0) ? 0.85 : 1.15;
    xAxis->scaleRange(factor, xAxis->pixelToCoord(event->position().toPoint().x()));
    yAxis->scaleRange(factor, yAxis->pixelToCoord(event->position().toPoint().y()));
    
    emit view_changed(xAxis->range().lower, xAxis->range().upper);
    replot();
}

void AdvancedTimeSeriesChart::mousePressEvent(QMouseEvent* event) {
    if (!interactive_enabled_) return;
    
    is_panning_ = true;
    last_mouse_ = event->pos();
}

void AdvancedTimeSeriesChart::mouseMoveEvent(QMouseEvent* event) {
    if (!is_panning_ || !interactive_enabled_) return;
    
    int delta_x = event->pos().x() - last_mouse_.x();
    int delta_y = event->pos().y() - last_mouse_.y();
    
    xAxis->moveRange(-delta_x * xAxis->range().size() / width());
    yAxis->moveRange(delta_y * yAxis->range().size() / height());
    
    last_mouse_ = event->pos();
    replot();
}

void AdvancedTimeSeriesChart::mouseReleaseEvent(QMouseEvent* event) {
    Q_UNUSED(event);
    is_panning_ = false;
}

double AdvancedTimeSeriesChart::calc_mean(const QVector<double>& data) const {
    if (data.isEmpty()) return 0;
    return std::accumulate(data.begin(), data.end(), 0.0) / data.size();
}

double AdvancedTimeSeriesChart::calc_stddev(const QVector<double>& data) const {
    if (data.size() < 2) return 0;
    double mean = calc_mean(data);
    double sum_sq_diff = 0;
    for (double v : data) {
        sum_sq_diff += (v - mean) * (v - mean);
    }
    return std::sqrt(sum_sq_diff / (data.size() - 1));
}

void AdvancedTimeSeriesChart::linear_regression(const QVector<double>& x,
                                               const QVector<double>& y,
                                               double& slope, double& intercept) const {
    if (x.size() < 2) {
        slope = 0;
        intercept = 0;
        return;
    }
    
    double mean_x = calc_mean(x);
    double mean_y = calc_mean(y);
    
    double numerator = 0, denominator = 0;
    for (int i = 0; i < x.size(); ++i) {
        numerator += (x[i] - mean_x) * (y[i] - mean_y);
        denominator += (x[i] - mean_x) * (x[i] - mean_x);
    }
    
    slope = (denominator != 0) ? numerator / denominator : 0;
    intercept = mean_y - slope * mean_x;
}

void AdvancedTimeSeriesChart::update_series_display(int series_id) {
    auto it = series_map_.find(series_id);
    if (it == series_map_.end()) return;
    
    SeriesData& series = it.value();
    
    if (series.anomaly_enabled) {
        double mean = calc_mean(series.values);
        double stddev = calc_stddev(series.values);
        double threshold = mean + (series.anomaly_threshold * stddev);
        
        for (int i = 0; i < series.values.size(); ++i) {
            if (series.values[i] > threshold) {
                emit anomaly_detected(series_id, series.times[i], series.values[i]);
            }
        }
    }
}
