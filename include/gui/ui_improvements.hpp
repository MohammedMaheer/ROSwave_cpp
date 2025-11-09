// ui_improvements.hpp
// World-class UI/UX enhancements for ROS2 Dashboard
// Design patterns and implementation guidelines

#pragma once

#include <QMainWindow>
#include <QWidget>
#include <QTimer>
#include <QPropertyAnimation>
#include <QGraphicsOpacityEffect>
#include <QMenu>
#include <QShortcut>
#include <QStandardItemModel>
#include <QSortFilterProxyModel>
#include <QLineEdit>
#include <QCheckBox>
#include <QComboBox>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QApplication>
#include <QClipboard>
#include <memory>

// ============================================================================
// 1. ANIMATED INDICATORS (Pulsing Health Status)
// ============================================================================

class AnimatedHealthIndicator : public QWidget {
    Q_OBJECT

private:
    enum class HealthStatus { HEALTHY, WARNING, CRITICAL };
    
    HealthStatus current_status_;
    QTimer animation_timer_;
    double pulse_intensity_;  // 0.0 to 1.0
    int pulse_direction_;     // +1 or -1

public:
    AnimatedHealthIndicator(QWidget* parent = nullptr)
        : QWidget(parent), current_status_(HealthStatus::HEALTHY),
          pulse_intensity_(0.5), pulse_direction_(1) {
        
        setFixedSize(24, 24);
        
        connect(&animation_timer_, &QTimer::timeout, this, [this]() {
            pulse_intensity_ += pulse_direction_ * 0.02;
            if (pulse_intensity_ >= 1.0 || pulse_intensity_ <= 0.5) {
                pulse_direction_ *= -1;
            }
            update();
        });
        
        animation_timer_.start(50);  // ~20 FPS for smooth animation
    }

    void set_status(HealthStatus status) {
        if (current_status_ != status) {
            current_status_ = status;
            if (status == HealthStatus::CRITICAL) {
                animation_timer_.setInterval(200);  // Faster pulse for critical
            } else if (status == HealthStatus::WARNING) {
                animation_timer_.setInterval(100);  // Medium pulse
            } else {
                animation_timer_.setInterval(50);   // Slow pulse
            }
            update();
        }
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        // Select color based on status
        QColor base_color;
        switch (current_status_) {
            case HealthStatus::HEALTHY:
                base_color = QColor(76, 175, 80);   // Green
                break;
            case HealthStatus::WARNING:
                base_color = QColor(255, 193, 7);   // Yellow
                break;
            case HealthStatus::CRITICAL:
                base_color = QColor(244, 67, 54);   // Red
                break;
        }

        // Draw outer glow (pulsing)
        QColor glow = base_color;
        glow.setAlpha(static_cast<int>(pulse_intensity_ * 100));
        painter.setBrush(glow);
        painter.setPen(Qt::NoPen);
        int glow_size = static_cast<int>(24 + pulse_intensity_ * 8);
        painter.drawEllipse((24 - glow_size) / 2, (24 - glow_size) / 2,
                           glow_size, glow_size);

        // Draw main circle
        painter.setBrush(base_color);
        painter.drawEllipse(2, 2, 20, 20);

        // Draw border
        painter.setPen(QPen(base_color.darker(), 1));
        painter.setBrush(Qt::NoBrush);
        painter.drawEllipse(2, 2, 20, 20);
    }
};

// ============================================================================
// 2. SMOOTH CHART TRANSITIONS WITH ANIMATIONS
// ============================================================================

class AnimatedChartUpdater : public QObject {
    Q_OBJECT

private:
    QPropertyAnimation* animation_;
    std::vector<double> old_values_;
    std::vector<double> new_values_;
    std::vector<double> current_values_;

public:
    AnimatedChartUpdater(QObject* parent = nullptr)
        : QObject(parent), animation_(nullptr) {
    }

    void animate_chart_update(const std::vector<double>& new_data,
                             int duration_ms = 500) {
        old_values_ = current_values_;
        new_values_ = new_data;
        current_values_.resize(new_data.size(), 0.0);

        if (animation_) {
            animation_->stop();
            delete animation_;
        }

        animation_ = new QPropertyAnimation(this, "chart_progress");
        animation_->setDuration(duration_ms);
        animation_->setStartValue(0.0);
        animation_->setEndValue(1.0);
        animation_->setEasingCurve(QEasingCurve::InOutQuad);

        connect(animation_, &QPropertyAnimation::valueChanged, this,
                [this](const QVariant& value) {
                    double progress = value.toDouble();
                    for (size_t i = 0; i < new_values_.size(); ++i) {
                        current_values_[i] =
                            old_values_[i] +
                            (new_values_[i] - old_values_[i]) * progress;
                    }
                    emit chart_updated(current_values_);
                });

        animation_->start();
    }

    Q_PROPERTY(double chart_progress READ get_progress WRITE set_progress)

    double get_progress() const { return 0.0; }
    void set_progress(double) {}

signals:
    void chart_updated(const std::vector<double>& values);
};

// ============================================================================
// 3. ADVANCED SEARCH & FILTER SYSTEM
// ============================================================================

class AdvancedTopicFilter : public QWidget {
    Q_OBJECT

private:
    QLineEdit* search_input_;
    QComboBox* filter_type_;
    QCheckBox* regex_mode_;
    QStandardItemModel* search_history_;

public:
    AdvancedTopicFilter(QWidget* parent = nullptr)
        : QWidget(parent) {
        
        auto layout = new QVBoxLayout(this);

        // Search input with autocomplete
        search_input_ = new QLineEdit;
        search_input_->setPlaceholderText(
            "Search topics (Ctrl+F) - Use regex with 'r:pattern'");
        layout->addWidget(search_input_);

        // Filter options
        auto filter_layout = new QHBoxLayout;
        
        filter_type_ = new QComboBox;
        filter_type_->addItems({"All Fields", "Name", "Type", "Rate"});
        filter_layout->addWidget(new QLabel("Filter by:"));
        filter_layout->addWidget(filter_type_);

        regex_mode_ = new QCheckBox("Regular Expression");
        filter_layout->addWidget(regex_mode_);
        filter_layout->addStretch();
        
        layout->addLayout(filter_layout);

        // Connect signals
        connect(search_input_, &QLineEdit::textChanged, this,
                &AdvancedTopicFilter::on_search_text_changed);
        connect(search_input_, &QLineEdit::returnPressed, this,
                &AdvancedTopicFilter::save_search_history);
    }

    struct FilterCriteria {
        QString search_text;
        QString filter_field;
        bool use_regex = false;
    };

    FilterCriteria get_filter_criteria() const {
        return {
            search_input_->text(),
            filter_type_->currentText(),
            regex_mode_->isChecked()
        };
    }

    void add_search_history_item(const QString& query) {
        if (search_history_->rowCount() > 20) {
            search_history_->removeRow(0);
        }
        search_history_->appendRow(new QStandardItem(query));
    }

private slots:
    void on_search_text_changed(const QString& text) {
        emit filter_changed(get_filter_criteria());
    }

    void save_search_history() {
        if (!search_input_->text().isEmpty()) {
            add_search_history_item(search_input_->text());
            emit search_executed(search_input_->text());
        }
    }

signals:
    void filter_changed(const FilterCriteria& criteria);
    void search_executed(const QString& query);
};

// ============================================================================
// 4. CONTEXT MENUS WITH RICH ACTIONS
// ============================================================================

class TopicContextMenu : public QMenu {
    Q_OBJECT

public:
    TopicContextMenu(const QString& topic_name, QWidget* parent = nullptr)
        : QMenu(parent) {
        
        setWindowTitle(QString("Topic: %1").arg(topic_name));

        // Copy actions
        addAction("📋 Copy Name", [this, topic_name]() {
            QApplication::clipboard()->setText(topic_name);
        });

        addAction("📋 Copy Full Type", [this, topic_name]() {
            // Get full type and copy
            QString full_type = get_topic_full_type(topic_name);
            QApplication::clipboard()->setText(full_type);
        });

        addSeparator();

        // Topic operations
        addAction("👁️ Subscribe (Inspector)", [this, topic_name]() {
            emit subscribe_to_topic(topic_name);
        });

        addAction("⏺️ Record This Topic", [this, topic_name]() {
            emit record_topic(topic_name);
        });

        addAction("📊 Add to Monitoring", [this, topic_name]() {
            emit add_to_monitoring(topic_name);
        });

        addSeparator();

        // Information
        addAction("ℹ️ View Message Definition", [this, topic_name]() {
            show_message_definition(topic_name);
        });

        addAction("📈 View Statistics", [this, topic_name]() {
            show_topic_statistics(topic_name);
        });
    }

signals:
    void subscribe_to_topic(const QString& topic_name);
    void record_topic(const QString& topic_name);
    void add_to_monitoring(const QString& topic_name);

private:
    QString get_topic_full_type(const QString& topic_name) {
        // Get from ROS2 manager
        return "sensor_msgs/msg/PointCloud2";  // Example
    }

    void show_message_definition(const QString& topic_name) {
        // Show message definition in a dialog
    }

    void show_topic_statistics(const QString& topic_name) {
        // Show statistics dialog
    }
};

// ============================================================================
// 5. KEYBOARD SHORTCUTS MANAGER
// ============================================================================

class KeyboardShortcutsManager : public QObject {
    Q_OBJECT

public:
    struct Shortcut {
        QString key_sequence;
        QString description;
        std::function<void()> action;
    };

    static KeyboardShortcutsManager& instance() {
        static KeyboardShortcutsManager instance_;
        return instance_;
    }

    void register_shortcut(const QString& sequence, const QString& description,
                         std::function<void()> action) {
        auto shortcut = new QShortcut(QKeySequence(sequence), qApp->activeWindow());
        connect(shortcut, &QShortcut::activated, this,
                [action]() { action(); });

        shortcuts_.push_back({sequence, description, action});
    }

    void setup_dashboard_shortcuts(QMainWindow* main_window) {
        // Tab navigation
        new QShortcut(Qt::CTRL + Qt::Key_T, main_window,
                     SLOT(focus_topics_tab()));
        new QShortcut(Qt::CTRL + Qt::Key_N, main_window,
                     SLOT(focus_nodes_tab()));
        new QShortcut(Qt::CTRL + Qt::Key_S, main_window,
                     SLOT(focus_services_tab()));

        // Operations
        new QShortcut(Qt::CTRL + Qt::Key_R, main_window,
                     SLOT(refresh_all()));
        new QShortcut(Qt::CTRL + Qt::Key_F, main_window,
                     SLOT(focus_search()));
        new QShortcut(Qt::CTRL + Qt::Key_E, main_window,
                     SLOT(export_data()));
        new QShortcut(Qt::CTRL + Qt::Key_L, main_window,
                     SLOT(toggle_theme()));

        // Recording
        new QShortcut(Qt::Key_Space, main_window,
                     SLOT(toggle_recording()));

        // Help
        new QShortcut(Qt::CTRL + Qt::Key_Question, main_window,
                     SLOT(show_keyboard_shortcuts()));
    }

    void show_shortcuts_dialog(QWidget* parent) {
        QDialog dialog(parent);
        dialog.setWindowTitle("Keyboard Shortcuts");
        dialog.setMinimumSize(500, 400);

        auto layout = new QVBoxLayout(&dialog);
        auto table = new QTableWidget;
        table->setColumnCount(2);
        table->setHorizontalHeaderLabels({"Shortcut", "Action"});
        table->horizontalHeader()->setStretchLastSection(true);

        int row = 0;
        for (const auto& sc : shortcuts_) {
            table->insertRow(row);
            table->setItem(row, 0, new QTableWidgetItem(sc.key_sequence));
            table->setItem(row, 1, new QTableWidgetItem(sc.description));
            row++;
        }

        layout->addWidget(table);

        QPushButton close_btn("Close");
        connect(&close_btn, &QPushButton::clicked, &dialog, &QDialog::accept);
        layout->addWidget(&close_btn);

        dialog.exec();
    }

private slots:
    // Dummy slots for shortcuts
    void focus_topics_tab() {}
    void focus_nodes_tab() {}
    void focus_services_tab() {}
    void refresh_all() {}
    void focus_search() {}
    void export_data() {}
    void toggle_theme() {}
    void toggle_recording() {}
    void show_keyboard_shortcuts() {}

private:
    std::vector<Shortcut> shortcuts_;
};

// ============================================================================
// 6. DOCKABLE PANELS WITH LAYOUT PERSISTENCE
// ============================================================================

class DockablePanelManager : public QObject {
    Q_OBJECT

private:
    QMainWindow* main_window_;
    QSettings settings_;

public:
    DockablePanelManager(QMainWindow* window)
        : main_window_(window), settings_("ROS2Dashboard", "LayoutPresets") {
    }

    void save_layout(const QString& preset_name = "default") {
        QByteArray geometry = main_window_->saveGeometry();
        QByteArray state = main_window_->saveState();

        settings_.setValue(QString("geometry_%1").arg(preset_name), geometry);
        settings_.setValue(QString("state_%1").arg(preset_name), state);
    }

    void restore_layout(const QString& preset_name = "default") {
        QByteArray geometry = settings_.value(
            QString("geometry_%1").arg(preset_name)).toByteArray();
        QByteArray state = settings_.value(
            QString("state_%1").arg(preset_name)).toByteArray();

        if (!geometry.isEmpty()) {
            main_window_->restoreGeometry(geometry);
        }
        if (!state.isEmpty()) {
            main_window_->restoreState(state);
        }
    }

    void create_dockable_tab(const QString& tab_name, QWidget* content) {
        auto dock = new QDockWidget(tab_name, main_window_);
        dock->setWidget(content);
        dock->setObjectName(tab_name);
        dock->setFeatures(QDockWidget::DockWidgetMovable |
                         QDockWidget::DockWidgetFloatable);
        main_window_->addDockWidget(Qt::RightDockWidgetArea, dock);
    }

    QStringList get_saved_presets() const {
        QStringList presets;
        for (const auto& key : settings_.allKeys()) {
            if (key.startsWith("geometry_")) {
                presets << key.mid(9);  // Remove "geometry_" prefix
            }
        }
        return presets;
    }
};

// ============================================================================
// 7. THEME MANAGEMENT (Light/Dark Mode)
// ============================================================================

class ThemeManager : public QObject {
    Q_OBJECT

public:
    enum class Theme { LIGHT, DARK, AUTO };

    static ThemeManager& instance() {
        static ThemeManager instance_;
        return instance_;
    }

    void set_theme(Theme theme) {
        QString stylesheet = (theme == Theme::DARK) ? get_dark_stylesheet()
                                                     : get_light_stylesheet();
        qApp->setStyle(new QProxyStyle(qApp->style()));
        qApp->setStyleSheet(stylesheet);
        current_theme_ = theme;
        emit theme_changed(theme);
    }

    Theme get_current_theme() const { return current_theme_; }

    void toggle_theme() {
        set_theme(current_theme_ == Theme::DARK ? Theme::LIGHT : Theme::DARK);
    }

private:
    Theme current_theme_ = Theme::LIGHT;

    QString get_light_stylesheet() const {
        return R"(
            QMainWindow {
                background-color: #FFFFFF;
                color: #000000;
            }
            QWidget {
                background-color: #F5F5F5;
                color: #000000;
            }
            QHeaderView::section {
                background-color: #E0E0E0;
                color: #000000;
                padding: 5px;
                border: 1px solid #BDBDBD;
            }
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                padding: 5px 10px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #1565C0;
            }
        )";
    }

    QString get_dark_stylesheet() const {
        return R"(
            QMainWindow {
                background-color: #1E1E1E;
                color: #FFFFFF;
            }
            QWidget {
                background-color: #2D2D2D;
                color: #FFFFFF;
            }
            QHeaderView::section {
                background-color: #3F3F3F;
                color: #FFFFFF;
                padding: 5px;
                border: 1px solid #555555;
            }
            QPushButton {
                background-color: #1976D2;
                color: white;
                border: none;
                padding: 5px 10px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #2196F3;
            }
            QPushButton:pressed {
                background-color: #1565C0;
            }
        )";
    }

signals:
    void theme_changed(Theme theme);
};

// ============================================================================
// INTEGRATION EXAMPLE
// ============================================================================

/*
class EnhancedDashboard : public QMainWindow {
public:
    EnhancedDashboard() {
        // Setup keyboard shortcuts
        KeyboardShortcutsManager::instance().setup_dashboard_shortcuts(this);

        // Setup theme
        ThemeManager::instance().set_theme(ThemeManager::Theme::DARK);

        // Setup dockable panels
        DockablePanelManager dock_mgr(this);
        dock_mgr.create_dockable_tab("Topics", new QWidget);
        dock_mgr.restore_layout("default");

        // Setup animations
        chart_updater_ = new AnimatedChartUpdater(this);

        // Add context menus to topic list
        topic_list_->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(topic_list_, &QWidget::customContextMenuRequested,
                this, &EnhancedDashboard::show_topic_context_menu);
    }

private:
    AnimatedChartUpdater* chart_updater_;

    void show_topic_context_menu(const QPoint& pos) {
        auto menu = new TopicContextMenu("sensor/camera", this);
        menu->popup(QCursor::pos());
    }
};
*/

