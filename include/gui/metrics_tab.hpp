/**
 * @file gui/metrics_tab.hpp
 * @brief Real-time metrics dashboard with interactive charts
 * @author Dashboard Team
 */

#pragma once

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <memory>
#include "metrics_collector.hpp"
#include "metrics_history_buffer.hpp"
#include "async_worker.hpp"
#include "../ros2_metrics_collector.hpp"

#ifdef HAVE_QCUSTOMPLOT
    #include "qcustomplot.h"
#endif

namespace ros2_dashboard::gui {

/**
 * @class MetricsTab
 * @brief Displays real-time system metrics with interactive charts
 * 
 * Features:
 * - 4 live charts: CPU %, Memory MB, Disk I/O MB/s, Network Mbps
 * - Historical data visualization (up to 10 minutes)
 * - Auto-scaling Y-axis based on data range
 * - Real-time updates (1Hz)
 * - Export capabilities (PNG chart, CSV data)
 * - Statistics panel (min/max/avg)
 */
class MetricsTab : public QWidget {
    Q_OBJECT

public:
    explicit MetricsTab(QWidget* parent = nullptr);
    ~MetricsTab() override;

    void initialize(std::shared_ptr<MetricsCollector> metrics_collector,
                   std::shared_ptr<AsyncWorker> async_worker,
                   std::shared_ptr<ros2_dashboard::ROS2MetricsCollector> ros2_collector = nullptr);

    void refresh_metrics();

private slots:
    void on_export_csv_clicked();
    void on_export_json_clicked();
    void on_export_chart_clicked();
    void on_refresh_history_clicked();
    void on_update_timer_timeout();

private:
    std::shared_ptr<MetricsCollector> metrics_collector_;
    std::shared_ptr<AsyncWorker> async_worker_;
    std::shared_ptr<ros2_dashboard::ROS2MetricsCollector> ros2_metrics_collector_;
    std::unique_ptr<MetricsHistoryBuffer> history_buffer_;
    QTimer* metrics_refresh_timer_;

    // Chart widgets (if QCustomPlot available)
#ifdef HAVE_QCUSTOMPLOT
    QCustomPlot* cpu_chart_;
    QCustomPlot* memory_chart_;
    QCustomPlot* disk_io_chart_;
    QCustomPlot* network_chart_;
    QCustomPlot* topics_chart_;
    QCustomPlot* throughput_chart_;
    QCustomPlot* service_latency_chart_;
    QCustomPlot* recording_chart_;
#endif

    // Stat labels
    QLabel* cpu_label_;
    QLabel* memory_label_;
    QLabel* disk_label_;
    QLabel* thermal_label_;
    QLabel* write_speed_label_;
    QLabel* frame_rate_label_;
    QLabel* cache_ratio_label_;

    // Stat displays (min/max/avg)
    QLabel* cpu_stats_label_;
    QLabel* memory_stats_label_;
    QLabel* disk_stats_label_;
    QLabel* network_stats_label_;
    QLabel* node_count_label_;
    QLabel* service_count_label_;
    QLabel* messages_per_sec_label_;

    // UI container for scrolling
    class QScrollArea* scroll_area_;

    void create_ui_();
    void update_metrics_display_();
    void update_charts_();
    void update_ros2_charts_();
    void on_maximize_button_clicked();
#ifdef HAVE_QCUSTOMPLOT
    void setup_chart_(QCustomPlot* chart, const QString& title, const QString& y_label);
    void update_chart_(QCustomPlot* chart, 
                     const std::vector<double>& x_data,
                     const std::vector<double>& y_data,
                     const QColor& color);
#endif
};

}  // namespace ros2_dashboard::gui
