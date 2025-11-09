#pragma once

#include <qcustomplot.h>
#include <QVector>
#include <QMap>
#include <QString>
#include <QColor>
#include <memory>

/**
 * @brief Advanced charting widget with multiple axes, zoom, pan, and forecasting
 * 
 * Features:
 * - Multiple independent Y-axes (e.g., temperature + humidity)
 * - Synchronized X-axis (time) across all plots
 * - Zoom with mouse wheel
 * - Pan with mouse drag
 * - Trend line with linear regression
 * - Trend forecasting
 * - Anomaly detection and highlighting
 * - Peak detection
 * - Configurable time range
 * - Export as PNG or CSV
 */
class AdvancedTimeSeriesChart : public QCustomPlot {
    Q_OBJECT

public:
    enum class TimeRange { ONE_HOUR, SIX_HOURS, ONE_DAY, ONE_WEEK, ALL };

    explicit AdvancedTimeSeriesChart(QWidget* parent = nullptr);
    ~AdvancedTimeSeriesChart() = default;

    // Series management
    int add_series(const QString& series_name, const QColor& color);
    void add_data_point(int series_id, double timestamp, double value);
    void clear_series(int series_id);
    void remove_series(int series_id);
    void toggle_series_visibility(int series_id);

    // Statistics
    struct SeriesStats {
        double min, max, avg, latest;
        int point_count;
    };
    SeriesStats get_series_stats(int series_id) const;

    // Visualization features
    void set_show_trend_line(int series_id, bool enabled);
    void set_show_forecast(int series_id, bool enabled);
    void set_forecast_horizon(int seconds);
    void set_anomaly_detection(int series_id, bool enabled, double threshold = 2.0);
    void set_show_peaks(int series_id, bool enabled);
    void set_peak_sensitivity(int series_id, double sensitivity);

    // Display control
    void set_time_range(TimeRange range);
    void set_legend_toggling_enabled(bool enabled);
    void set_interactive_enabled(bool enabled);
    void fit_to_data();
    void refresh();

    // Export
    bool export_as_image(const QString& file_path, int width = 1280, int height = 720);
    bool export_as_csv(const QString& file_path) const;

    // Query
    QPair<double, double> get_visible_time_range() const;

signals:
    void time_range_changed(TimeRange range);
    void view_changed(double x_min, double x_max);
    void anomaly_detected(int series_id, double timestamp, double value);

protected:
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    struct SeriesData {
        int id;
        QString name;
        QCPGraph* graph;
        QVector<double> times;
        QVector<double> values;
        bool show_trend = false;
        bool show_forecast = false;
        bool anomaly_enabled = false;
        bool show_peaks = false;
        double anomaly_threshold = 2.0;
        double peak_sensitivity = 0.5;
    };

    void setup_axes();
    void update_series_display(int series_id);
    double calc_mean(const QVector<double>& data) const;
    double calc_stddev(const QVector<double>& data) const;
    void linear_regression(const QVector<double>& x, const QVector<double>& y,
                          double& slope, double& intercept) const;

    QMap<int, SeriesData> series_map_;
    int next_series_id_;
    TimeRange time_range_;
    int forecast_seconds_;
    bool legend_toggle_enabled_;
    bool interactive_enabled_;
    bool is_panning_;
    QPoint last_mouse_;
};

