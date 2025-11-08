/**
 * @file gui/metrics_tab.cpp
 * @brief Implementation of metrics dashboard with charts
 */

#include "gui/metrics_tab.hpp"
#include "metrics_history_buffer.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QFileDialog>
#include <QSplitter>
#include <Qt>
#include <algorithm>
#include <cmath>

namespace ros2_dashboard::gui {

MetricsTab::MetricsTab(QWidget* parent) 
    : QWidget(parent), 
      metrics_refresh_timer_(nullptr),
      cpu_label_(nullptr),
      memory_label_(nullptr),
      disk_label_(nullptr),
      thermal_label_(nullptr),
      write_speed_label_(nullptr),
      frame_rate_label_(nullptr),
      cache_ratio_label_(nullptr),
      cpu_stats_label_(nullptr),
      memory_stats_label_(nullptr),
      disk_stats_label_(nullptr),
      network_stats_label_(nullptr)
#ifdef HAVE_QCUSTOMPLOT
      , cpu_chart_(nullptr),
      memory_chart_(nullptr),
      disk_io_chart_(nullptr),
      network_chart_(nullptr)
#endif
{ 
    history_buffer_ = std::make_unique<MetricsHistoryBuffer>();
    create_ui_(); 
}

MetricsTab::~MetricsTab() = default;

void MetricsTab::initialize(std::shared_ptr<MetricsCollector> metrics_collector,
                           std::shared_ptr<AsyncWorker> async_worker) {
    metrics_collector_ = metrics_collector;
    async_worker_ = async_worker;
    
    // Setup auto-refresh timer
    metrics_refresh_timer_ = new QTimer(this);
    connect(metrics_refresh_timer_, &QTimer::timeout, this, &MetricsTab::on_update_timer_timeout);
    metrics_refresh_timer_->start(1000);  // Refresh every 1 second
}

void MetricsTab::refresh_metrics() {
    if (metrics_collector_) {
        // Call update directly in main thread - do NOT use async_worker
        // Qt widgets MUST be updated from the main thread only
        update_metrics_display_();
    }
}

void MetricsTab::on_export_csv_clicked() {
    QString filepath = QFileDialog::getSaveFileName(this,
        "Export Metrics as CSV", "", "CSV Files (*.csv)");
    if (!filepath.isEmpty() && metrics_collector_) {
        metrics_collector_->export_csv(filepath.toStdString(), 3600);
    }
}

void MetricsTab::on_export_json_clicked() {
    QString filepath = QFileDialog::getSaveFileName(this,
        "Export Metrics as JSON", "", "JSON Files (*.json)");
    if (!filepath.isEmpty() && metrics_collector_) {
        metrics_collector_->export_json(filepath.toStdString(), 3600);
    }
}

void MetricsTab::on_export_chart_clicked() {
#ifdef HAVE_QCUSTOMPLOT
    QString filepath = QFileDialog::getSaveFileName(this,
        "Export Chart as PNG", "", "PNG Files (*.png)");
    if (!filepath.isEmpty() && cpu_chart_) {
        cpu_chart_->savePng(filepath);
    }
#endif
}

void MetricsTab::on_refresh_history_clicked() {
    refresh_metrics();
}

void MetricsTab::on_update_timer_timeout() {
    refresh_metrics();
}

void MetricsTab::create_ui_() {
    auto main_layout = new QVBoxLayout();

    // ===== Current Metrics Display (Top Section) =====
    auto current_group = new QGroupBox("Current Metrics");
    auto current_layout = new QHBoxLayout();
    
    cpu_label_ = new QLabel("CPU: 0%");
    memory_label_ = new QLabel("Memory: 0 MB");
    disk_label_ = new QLabel("Disk: 0 MB free");
    thermal_label_ = new QLabel("Thermal: N/A");
    write_speed_label_ = new QLabel("Write Speed: 0 MB/s");
    frame_rate_label_ = new QLabel("Frame Rate: 0 FPS");
    cache_ratio_label_ = new QLabel("Cache Hit: 0%");
    
    current_layout->addWidget(cpu_label_);
    current_layout->addWidget(memory_label_);
    current_layout->addWidget(disk_label_);
    current_layout->addWidget(thermal_label_);
    current_layout->addWidget(write_speed_label_);
    current_layout->addWidget(frame_rate_label_);
    current_layout->addWidget(cache_ratio_label_);
    
    current_group->setLayout(current_layout);
    main_layout->addWidget(current_group);

    // ===== Statistics Display (Second Section) =====
    auto stats_group = new QGroupBox("Statistics (min/max/avg)");
    auto stats_layout = new QHBoxLayout();
    
    cpu_stats_label_ = new QLabel("CPU: N/A");
    memory_stats_label_ = new QLabel("Memory: N/A");
    disk_stats_label_ = new QLabel("Disk I/O: N/A");
    network_stats_label_ = new QLabel("Network: N/A");
    
    stats_layout->addWidget(cpu_stats_label_);
    stats_layout->addWidget(memory_stats_label_);
    stats_layout->addWidget(disk_stats_label_);
    stats_layout->addWidget(network_stats_label_);
    
    stats_group->setLayout(stats_layout);
    main_layout->addWidget(stats_group);

#ifdef HAVE_QCUSTOMPLOT
    // ===== Charts Display (Third Section) - Lazy loaded =====
    auto charts_group = new QGroupBox("Live Metrics Charts (Last 10 minutes)");
    auto charts_layout = new QGridLayout();
    
    // Create 4 charts in 2x2 grid
    cpu_chart_ = new QCustomPlot();
    setup_chart_(cpu_chart_, "CPU Usage", "Percentage (%)");
    
    memory_chart_ = new QCustomPlot();
    setup_chart_(memory_chart_, "Memory Usage", "Megabytes (MB)");
    
    disk_io_chart_ = new QCustomPlot();
    setup_chart_(disk_io_chart_, "Disk I/O", "MB/s");
    
    network_chart_ = new QCustomPlot();
    setup_chart_(network_chart_, "Network", "Mbps");
    
    charts_layout->addWidget(cpu_chart_, 0, 0);
    charts_layout->addWidget(memory_chart_, 0, 1);
    charts_layout->addWidget(disk_io_chart_, 1, 0);
    charts_layout->addWidget(network_chart_, 1, 1);
    
    charts_group->setLayout(charts_layout);
    main_layout->addWidget(charts_group);
#endif

    // ===== Export Buttons (Bottom Section) =====
    auto export_layout = new QHBoxLayout();
    auto export_csv_btn = new QPushButton("Export CSV");
    auto export_json_btn = new QPushButton("Export JSON");
    auto refresh_btn = new QPushButton("Refresh");
    
    connect(export_csv_btn, &QPushButton::clicked,
            this, &MetricsTab::on_export_csv_clicked);
    connect(export_json_btn, &QPushButton::clicked,
            this, &MetricsTab::on_export_json_clicked);
    connect(refresh_btn, &QPushButton::clicked,
            this, &MetricsTab::on_refresh_history_clicked);
    
    export_layout->addWidget(export_csv_btn);
    export_layout->addWidget(export_json_btn);
    export_layout->addWidget(refresh_btn);
    export_layout->addStretch();
    
    main_layout->addLayout(export_layout);
    setLayout(main_layout);
}

void MetricsTab::update_metrics_display_() {
    if (!metrics_collector_) return;

    // Get current metrics
    auto snapshot = metrics_collector_->get_snapshot();
    auto sys = snapshot.system;
    auto rec = snapshot.recording;
    auto app = snapshot.application;

    // Update current value labels
    cpu_label_->setText(QString::asprintf("CPU: %.1f%%", sys.cpu_usage_percent));
    memory_label_->setText(QString::asprintf("Memory: %ld MB", sys.memory_usage_mb));
    disk_label_->setText(QString::asprintf("Disk: %ld MB free", sys.disk_free_mb));
    thermal_label_->setText(QString::asprintf("Thermal: %dC", sys.temperature_celsius));
    write_speed_label_->setText(QString::asprintf("Write Speed: %.1f MB/s", rec.write_speed_mbps));
    frame_rate_label_->setText(QString::asprintf("Frame Rate: %.1f FPS", app.ui_frame_rate_fps));
    cache_ratio_label_->setText(QString::asprintf("Cache Hit: %.1f%%", app.cache_hit_ratio * 100));

    // Add to history buffer
    MetricsPoint pt;
    pt.timestamp_ms = snapshot.timestamp_ms;
    pt.cpu_percent = sys.cpu_usage_percent;
    pt.memory_mb = sys.memory_usage_mb;
    pt.disk_io_mbps = rec.write_speed_mbps;
    pt.network_mbps = snapshot.network.bandwidth_mbps;
    pt.temperature_c = sys.temperature_celsius;
    pt.frame_rate_fps = app.ui_frame_rate_fps;
    pt.cache_ratio = app.cache_hit_ratio;
    
    history_buffer_->push_point(std::move(pt));

    // Update statistics
    auto cpu_stats = history_buffer_->get_cpu_stats();
    auto mem_stats = history_buffer_->get_memory_stats();
    auto disk_stats = history_buffer_->get_disk_io_stats();
    auto net_stats = history_buffer_->get_network_stats();

    cpu_stats_label_->setText(
        QString::asprintf("CPU: min=%.1f%% max=%.1f%% avg=%.1f%%",
                         cpu_stats.min, cpu_stats.max, cpu_stats.avg));
    memory_stats_label_->setText(
        QString::asprintf("Memory: min=%.0f MB max=%.0f MB avg=%.0f MB",
                         mem_stats.min, mem_stats.max, mem_stats.avg));
    disk_stats_label_->setText(
        QString::asprintf("Disk I/O: min=%.1f max=%.1f avg=%.1f MB/s",
                         disk_stats.min, disk_stats.max, disk_stats.avg));
    network_stats_label_->setText(
        QString::asprintf("Network: min=%.1f max=%.1f avg=%.1f Mbps",
                         net_stats.min, net_stats.max, net_stats.avg));

    // Update charts
    update_charts_();
}

void MetricsTab::update_charts_() {
#ifdef HAVE_QCUSTOMPLOT
    if (!history_buffer_) return;
    
    // Get all historical points
    auto points = history_buffer_->get_all_points();
    if (points.empty()) return;
    
    // Prepare data for charts
    std::vector<double> x_data;  // Time axis (seconds from start)
    std::vector<double> cpu_data;
    std::vector<double> memory_data;
    std::vector<double> disk_data;
    std::vector<double> network_data;
    
    if (!points.empty()) {
        uint64_t start_time = points.front().timestamp_ms;
        
        for (const auto& pt : points) {
            double elapsed_sec = (pt.timestamp_ms - start_time) / 1000.0;
            x_data.push_back(elapsed_sec);
            cpu_data.push_back(pt.cpu_percent);
            memory_data.push_back(static_cast<double>(pt.memory_mb));
            disk_data.push_back(pt.disk_io_mbps);
            network_data.push_back(pt.network_mbps);
        }
    }
    
    // Update each chart
    if (cpu_chart_) update_chart_(cpu_chart_, x_data, cpu_data, Qt::blue);
    if (memory_chart_) update_chart_(memory_chart_, x_data, memory_data, Qt::green);
    if (disk_io_chart_) update_chart_(disk_io_chart_, x_data, disk_data, Qt::red);
    if (network_chart_) update_chart_(network_chart_, x_data, network_data, Qt::darkMagenta);
#endif
}

#ifdef HAVE_QCUSTOMPLOT

void MetricsTab::setup_chart_(QCustomPlot* chart, const QString& title, const QString& y_label) {
    if (!chart) return;

    chart->setWindowTitle(title);
    chart->setMinimumHeight(250);

    // Setup title
    chart->plotLayout()->insertRow(0);
    chart->plotLayout()->addElement(0, 0, new QCPTextElement(chart, title, QFont("sans", 10, QFont::Bold)));

    // Setup axes
    chart->xAxis->setLabel("Time (seconds)");
    chart->yAxis->setLabel(y_label);

    // Setup grid
    chart->xAxis->grid()->setVisible(true);
    chart->yAxis->grid()->setVisible(true);

    // Setup appearance
    chart->setBackground(Qt::white);
    chart->axisRect()->setBackground(QBrush(QColor(240, 240, 240)));

    // Create data curve
    chart->addGraph();
    chart->graph(0)->setPen(QPen(Qt::blue, 1.5));
    chart->graph(0)->setAdaptiveSampling(true);  // Performance optimization
}

void MetricsTab::update_chart_(QCustomPlot* chart, 
                               const std::vector<double>& x_data,
                               const std::vector<double>& y_data,
                               const QColor& color) {
    if (!chart || x_data.empty() || y_data.empty() || x_data.size() != y_data.size()) {
        return;
    }

    // Update graph data
    if (chart->graphCount() == 0) {
        chart->addGraph();
    }
    
    QVector<double> qx_data(x_data.begin(), x_data.end());
    QVector<double> qy_data(y_data.begin(), y_data.end());
    chart->graph(0)->setData(qx_data, qy_data);
    chart->graph(0)->setPen(QPen(color, 1.5));

    // Auto scale to fit data
    chart->xAxis->rescale();
    chart->yAxis->rescale();
    
    // Add some padding to y-axis
    double y_range = chart->yAxis->range().size();
    chart->yAxis->setRange(chart->yAxis->range().lower - y_range * 0.05,
                          chart->yAxis->range().upper + y_range * 0.05);

    // Redraw
    chart->replot();
}

#endif  // HAVE_QCUSTOMPLOT

}  // namespace ros2_dashboard::gui
