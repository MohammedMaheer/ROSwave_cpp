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
#include <QScrollArea>
#include <QMainWindow>
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
    network_stats_label_(nullptr),
    node_count_label_(nullptr),
    service_count_label_(nullptr),
    messages_per_sec_label_(nullptr),
    ros2_metrics_collector_(nullptr)
#ifdef HAVE_QCUSTOMPLOT
      , cpu_chart_(nullptr),
      memory_chart_(nullptr),
      disk_io_chart_(nullptr),
      network_chart_(nullptr)
    , topics_chart_(nullptr),
    throughput_chart_(nullptr),
    service_latency_chart_(nullptr),
    recording_chart_(nullptr)
#endif
{ 
    history_buffer_ = std::make_unique<MetricsHistoryBuffer>();
    create_ui_(); 
}

MetricsTab::~MetricsTab() = default;

void MetricsTab::initialize(std::shared_ptr<MetricsCollector> metrics_collector,
                           std::shared_ptr<AsyncWorker> async_worker,
                           std::shared_ptr<ros2_dashboard::ROS2MetricsCollector> ros2_collector) {
    metrics_collector_ = metrics_collector;
    async_worker_ = async_worker;
    ros2_metrics_collector_ = ros2_collector;
    
    // Initialize AlertManager
    alert_manager_ = std::make_shared<AlertManager>();
    
    // Set up callback when alerts are raised
    alert_manager_->on_alert([this](const Alert& alert) {
        // Use Qt's signal/slot mechanism for thread-safe UI updates
        QMetaObject::invokeMethod(this, [this, alert]() {
            on_alert_received_(alert);
        }, Qt::QueuedConnection);
    });
    
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
    auto outer_layout = new QVBoxLayout(this);
    
    // ===== Top Control Bar with Maximize Button =====
    auto control_bar = new QHBoxLayout();
    auto maximize_btn = new QPushButton("⛶ Maximize Metrics Tab", this);
    connect(maximize_btn, &QPushButton::clicked, this, &MetricsTab::on_maximize_button_clicked);
    control_bar->addWidget(maximize_btn);
    control_bar->addStretch();
    outer_layout->addLayout(control_bar);
    
    // ===== Scrollable Content Area =====
    scroll_area_ = new QScrollArea(this);
    scroll_area_->setWidgetResizable(true);
    scroll_area_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll_area_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    
    // Create widget to hold all metrics content
    auto content_widget = new QWidget();
    auto main_layout = new QVBoxLayout(content_widget);

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
    
    // ROS2 specific summary labels
    node_count_label_ = new QLabel("Nodes: 0");
    service_count_label_ = new QLabel("Services: 0");
    messages_per_sec_label_ = new QLabel("Msg/s: 0");
    stats_layout->addWidget(node_count_label_);
    stats_layout->addWidget(service_count_label_);
    stats_layout->addWidget(messages_per_sec_label_);
    
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

#ifdef HAVE_QCUSTOMPLOT
    // ===== ROS2 Charts (Fourth Section) - Topics/Throughput/Service/Recording =====
    auto ros2_charts_group = new QGroupBox("ROS2 Metrics Charts (Short History)");
    auto ros2_charts_layout = new QGridLayout();

    topics_chart_ = new QCustomPlot();
    setup_chart_(topics_chart_, "Topic Discovery Rate", "Topics/min");

    throughput_chart_ = new QCustomPlot();
    setup_chart_(throughput_chart_, "Message Throughput", "Messages/s");

    service_latency_chart_ = new QCustomPlot();
    setup_chart_(service_latency_chart_, "Service Latency", "ms");

    recording_chart_ = new QCustomPlot();
    setup_chart_(recording_chart_, "Recording Throughput", "MB/s");

    ros2_charts_layout->addWidget(topics_chart_, 0, 0);
    ros2_charts_layout->addWidget(throughput_chart_, 0, 1);
    ros2_charts_layout->addWidget(service_latency_chart_, 1, 0);
    ros2_charts_layout->addWidget(recording_chart_, 1, 1);

    ros2_charts_group->setLayout(ros2_charts_layout);
    main_layout->addWidget(ros2_charts_group);
#endif

    // ===== Alerts Display Section =====
    auto alerts_group = new QGroupBox("Active Alerts");
    auto alerts_layout = new QVBoxLayout();
    
    alerts_status_label_ = new QLabel("Active Alerts: 0 Critical, 0 Warning");
    alerts_status_label_->setStyleSheet("QLabel { color: green; font-weight: bold; }");
    alerts_layout->addWidget(alerts_status_label_);
    
    alerts_list_ = new QListWidget();
    alerts_list_->setMaximumHeight(150);
    alerts_list_->setStyleSheet(
        "QListWidget { background-color: #f5f5f5; border: 1px solid #ccc; border-radius: 4px; }"
        "QListWidget::item { padding: 4px; }"
        "QListWidget::item:selected { background-color: #e0e0e0; }"
    );
    alerts_layout->addWidget(alerts_list_);
    
    alerts_group->setLayout(alerts_layout);
    main_layout->addWidget(alerts_group);

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
    content_widget->setLayout(main_layout);
    
    // Set the scrollable content
    scroll_area_->setWidget(content_widget);
    outer_layout->addWidget(scroll_area_);
    setLayout(outer_layout);
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

    // Update ROS2 summary labels if collector provided
    if (ros2_metrics_collector_) {
        auto ros2 = ros2_metrics_collector_->get_metrics();
        node_count_label_->setText(QString::asprintf("Nodes: %u", ros2.total_nodes));
        service_count_label_->setText(QString::asprintf("Services: %u", ros2.total_services));
        messages_per_sec_label_->setText(QString::asprintf("Msg/s: %llu", (unsigned long long)ros2.total_messages_per_sec));
    }

    // Check system metrics against alert thresholds
    if (alert_manager_) {
        // Get current timestamp in milliseconds
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        uint64_t timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
        
        // Check CPU threshold (default: 80%)
        if (sys.cpu_usage_percent > 80.0) {
            alert_manager_->raise_alert(Alert{
                AlertType::TOPIC_HIGH_RATE,  // Reuse existing enum
                AlertSeverity::WARNING,
                "system_cpu",
                QString::asprintf("High CPU usage: %.1f%%", sys.cpu_usage_percent).toStdString(),
                timestamp_ms,
                "MetricsTab"
            });
        }
        
        // Check memory threshold: if memory > 2GB, alert (approximate high memory)
        if (sys.memory_usage_mb > 2048) {
            alert_manager_->raise_alert(Alert{
                AlertType::TOPIC_HIGH_RATE,
                AlertSeverity::WARNING,
                "system_memory",
                QString::asprintf("High memory usage: %ld MB", sys.memory_usage_mb).toStdString(),
                timestamp_ms,
                "MetricsTab"
            });
        }
        
        // Check temperature threshold (default: 85C)
        if (sys.temperature_celsius > 85) {
            alert_manager_->raise_alert(Alert{
                AlertType::TOPIC_HIGH_RATE,
                AlertSeverity::CRITICAL,
                "system_thermal",
                QString::asprintf("High temperature: %dC", sys.temperature_celsius).toStdString(),
                timestamp_ms,
                "MetricsTab"
            });
        }
    }

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
    #ifdef HAVE_QCUSTOMPLOT
        // Update ROS2-specific charts if collector available
        update_ros2_charts_();
    #endif
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

void MetricsTab::update_ros2_charts_() {
#ifdef HAVE_QCUSTOMPLOT
    if (!ros2_metrics_collector_) return;

    auto topics = ros2_metrics_collector_->get_topic_discovery_history();
    auto throughput = ros2_metrics_collector_->get_message_throughput_history();
    auto servlat = ros2_metrics_collector_->get_service_latency_history();
    auto recording = ros2_metrics_collector_->get_recording_throughput_history();

    // Helper to build simple x axis (0..N-1 seconds)
    auto build_x = [](size_t n) {
        std::vector<double> x(n);
        for (size_t i = 0; i < n; ++i) x[i] = static_cast<double>(i);
        return x;
    };

    if (topics_chart_ && !topics.empty()) {
        auto x = build_x(topics.size());
        update_chart_(topics_chart_, x, topics, Qt::cyan);
    }

    if (throughput_chart_ && !throughput.empty()) {
        auto x = build_x(throughput.size());
        update_chart_(throughput_chart_, x, throughput, Qt::darkGreen);
    }

    if (service_latency_chart_ && !servlat.empty()) {
        auto x = build_x(servlat.size());
        update_chart_(service_latency_chart_, x, servlat, Qt::magenta);
    }

    if (recording_chart_ && !recording.empty()) {
        auto x = build_x(recording.size());
        update_chart_(recording_chart_, x, recording, Qt::darkCyan);
    }
#endif
}

void MetricsTab::on_maximize_button_clicked() {
    if (parentWidget() && parentWidget()->isWindow()) {
        auto* window = dynamic_cast<QMainWindow*>(parentWidget()->window());
        if (window) {
            if (window->isMaximized()) {
                window->showNormal();
            } else {
                window->showMaximized();
            }
        }
    }
}

void MetricsTab::on_alert_received_(const Alert& alert) {
    if (!alerts_list_) return;
    
    // Create visual representation of alert
    QString severity_icon;
    QString severity_color;
    switch (alert.severity) {
        case AlertSeverity::INFO:
            severity_icon = "ℹ️";
            severity_color = "blue";
            break;
        case AlertSeverity::WARNING:
            severity_icon = "⚠️";
            severity_color = "orange";
            break;
        case AlertSeverity::CRITICAL:
            severity_icon = "🔴";
            severity_color = "red";
            break;
    }
    
    // Format alert text
    QString alert_text = QString::fromStdString(
        severity_icon.toStdString() + " [" + alert.component + "] " + alert.message
    );
    
    // Add to list
    alerts_list_->insertItem(0, alert_text);
    
    // Keep only last 50 alerts
    while (alerts_list_->count() > 50) {
        delete alerts_list_->takeItem(alerts_list_->count() - 1);
    }
    
    // Update alert status label
    auto active_alerts = alert_manager_->get_active_alerts();
    int critical_count = 0, warning_count = 0;
    
    for (const auto& a : active_alerts) {
        if (a.severity == AlertSeverity::CRITICAL) critical_count++;
        else if (a.severity == AlertSeverity::WARNING) warning_count++;
    }
    
    QString status_text = QString::asprintf(
        "Active Alerts: %d Critical, %d Warning", critical_count, warning_count
    );
    
    if (alerts_status_label_) {
        alerts_status_label_->setText(status_text);
        
        // Color code the status label
        if (critical_count > 0) {
            alerts_status_label_->setStyleSheet("QLabel { color: red; font-weight: bold; }");
        } else if (warning_count > 0) {
            alerts_status_label_->setStyleSheet("QLabel { color: orange; font-weight: bold; }");
        } else {
            alerts_status_label_->setStyleSheet("QLabel { color: green; }");
        }
    }
}

}  // namespace ros2_dashboard::gui
