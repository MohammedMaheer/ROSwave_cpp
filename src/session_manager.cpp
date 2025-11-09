#include "../include/session_manager.hpp"
#include <QMainWindow>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <QStandardPaths>
#include <QDir>

SessionManager& SessionManager::instance() {
    static SessionManager instance_;
    return instance_;
}

SessionManager::SessionManager()
    : main_window_(nullptr), auto_save_enabled_(true),
      current_session_name_("default") {
}

void SessionManager::initialize(QMainWindow* main_window,
                               int auto_save_interval_ms) {
    main_window_ = main_window;

    // Initialize settings
    settings_ = std::make_unique<QSettings>(
        "ROS2Dashboard", "SessionManager", this);

    // Setup auto-save timer
    auto_save_timer_ = std::make_unique<QTimer>(this);
    connect(auto_save_timer_.get(), &QTimer::timeout, this,
            &SessionManager::on_auto_save_timeout);

    set_auto_save_interval(auto_save_interval_ms);

    // Setup crash detection
    setup_crash_detection();

    // Restore previous session
    restore_session();

    qInfo() << "SessionManager initialized";
}

void SessionManager::save_session() {
    if (!main_window_) return;

    emit before_auto_save();

    // Save window state
    save_window_state();

    // Save all preferences
    settings_->sync();

    emit after_auto_save();
    emit session_saved();

    qDebug() << "Session saved";
}

void SessionManager::restore_session() {
    if (!main_window_) return;

    // Check for crash
    if (has_crash_recovery_data()) {
        qWarning() << "Previous crash detected! Recovery data available.";
    }

    // Restore window state
    restore_window_state();

    mark_session_clean();
    emit session_restored();

    qDebug() << "Session restored";
}

void SessionManager::save_selected_topics(const QStringList& topics) {
    settings_->beginGroup(SESSION_GROUP);
    settings_->beginGroup(current_session_name_);
    settings_->setValue("selected_topics", topics);
    settings_->endGroup();
    settings_->endGroup();
    settings_->sync();
}

QStringList SessionManager::get_restored_topics() const {
    settings_->beginGroup(SESSION_GROUP);
    settings_->beginGroup(current_session_name_);
    QStringList topics = settings_->value("selected_topics", QStringList()).toStringList();
    settings_->endGroup();
    settings_->endGroup();
    return topics;
}

void SessionManager::save_window_state() {
    if (!main_window_) return;

    settings_->beginGroup("WindowState");
    settings_->setValue("geometry", main_window_->saveGeometry());
    settings_->setValue("window_state", main_window_->saveState());
    settings_->setValue("maximized", main_window_->isMaximized());
    settings_->endGroup();
    settings_->sync();

    qDebug() << "Window state saved";
}

void SessionManager::restore_window_state() {
    if (!main_window_) return;

    settings_->beginGroup("WindowState");
    QByteArray geometry = settings_->value("geometry", QByteArray()).toByteArray();
    QByteArray window_state = settings_->value("window_state", QByteArray()).toByteArray();
    bool maximized = settings_->value("maximized", false).toBool();
    settings_->endGroup();

    if (!geometry.isEmpty()) {
        main_window_->restoreGeometry(geometry);
    }
    if (!window_state.isEmpty()) {
        main_window_->restoreState(window_state);
    }
    if (maximized) {
        main_window_->showMaximized();
    }

    qDebug() << "Window state restored";
}

void SessionManager::set_preference(const QString& key, const QVariant& value) {
    settings_->setValue(get_settings_key(key), value);
}

QVariant SessionManager::get_preference(const QString& key,
                                       const QVariant& default_value) {
    return settings_->value(get_settings_key(key), default_value);
}

void SessionManager::save_recording_settings(const QString& output_dir,
                                            bool auto_compress) {
    settings_->beginGroup("Recording");
    settings_->setValue("last_directory", output_dir);
    settings_->setValue("auto_compress", auto_compress);
    settings_->endGroup();
    settings_->sync();
}

QString SessionManager::get_last_recording_dir() const {
    settings_->beginGroup("Recording");
    QString dir = settings_->value(
        "last_directory",
        QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation))
        .toString();
    settings_->endGroup();
    return dir;
}

void SessionManager::create_checkpoint(const QString& checkpoint_name,
                                       const QByteArray& data) {
    settings_->beginGroup("Checkpoints");
    settings_->setValue(checkpoint_name, data);
    settings_->endGroup();
    settings_->sync();

    qDebug() << "Checkpoint created:" << checkpoint_name;
}

QByteArray SessionManager::restore_checkpoint(const QString& checkpoint_name) {
    settings_->beginGroup("Checkpoints");
    QByteArray data = settings_->value(checkpoint_name, QByteArray()).toByteArray();
    settings_->endGroup();
    return data;
}

bool SessionManager::has_crash_recovery_data() const {
    return settings_->value(CRASH_MARKER, false).toBool();
}

QByteArray SessionManager::get_crash_recovery_data() {
    return restore_checkpoint("crash_recovery");
}

void SessionManager::mark_session_clean() {
    settings_->setValue(CRASH_MARKER, false);
    settings_->sync();
}

void SessionManager::enable_auto_save(bool enabled) {
    auto_save_enabled_ = enabled;
    if (enabled && auto_save_timer_) {
        auto_save_timer_->start();
    } else if (auto_save_timer_) {
        auto_save_timer_->stop();
    }
}

void SessionManager::set_auto_save_interval(int ms) {
    if (auto_save_timer_) {
        auto_save_timer_->setInterval(ms);
        if (auto_save_enabled_) {
            auto_save_timer_->start();
        }
    }
}

QString SessionManager::get_current_session_name() const {
    return current_session_name_;
}

void SessionManager::switch_session(const QString& session_name) {
    // Save current session first
    save_session();

    current_session_name_ = session_name;

    // Restore new session
    restore_session();

    qInfo() << "Switched to session:" << session_name;
}

QStringList SessionManager::list_sessions() const {
    settings_->beginGroup(SESSION_GROUP);
    QStringList sessions = settings_->childGroups();
    settings_->endGroup();
    return sessions;
}

void SessionManager::delete_session(const QString& session_name) {
    settings_->beginGroup(SESSION_GROUP);
    settings_->remove(session_name);
    settings_->endGroup();
    settings_->sync();

    qInfo() << "Session deleted:" << session_name;
}

bool SessionManager::export_session(const QString& file_path) {
    QJsonObject session_obj;

    // Export topics
    session_obj["topics"] = QJsonArray::fromStringList(get_restored_topics());

    // Export preferences
    settings_->beginGroup("Preferences");
    QJsonObject prefs;
    for (const auto& key : settings_->allKeys()) {
        prefs[key] = QJsonValue::fromVariant(settings_->value(key));
    }
    settings_->endGroup();
    session_obj["preferences"] = prefs;

    QJsonDocument doc(session_obj);
    QFile file(file_path);
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning() << "Failed to open file for export:" << file_path;
        return false;
    }

    file.write(doc.toJson());
    file.close();

    qInfo() << "Session exported to:" << file_path;
    return true;
}

bool SessionManager::import_session(const QString& file_path) {
    QFile file(file_path);
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "Failed to open file for import:" << file_path;
        return false;
    }

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    file.close();

    if (!doc.isObject()) {
        qWarning() << "Invalid session file format";
        return false;
    }

    QJsonObject obj = doc.object();

    // Import topics
    if (obj.contains("topics")) {
        QStringList topics;
        for (const auto& val : obj["topics"].toArray()) {
            topics.append(val.toString());
        }
        save_selected_topics(topics);
    }

    // Import preferences
    if (obj.contains("preferences")) {
        QJsonObject prefs = obj["preferences"].toObject();
        settings_->beginGroup("Preferences");
        for (auto it = prefs.begin(); it != prefs.end(); ++it) {
            settings_->setValue(it.key(), it.value().toVariant());
        }
        settings_->endGroup();
    }

    settings_->sync();
    qInfo() << "Session imported from:" << file_path;
    return true;
}

void SessionManager::on_auto_save_timeout() {
    save_session();
}

void SessionManager::setup_crash_detection() {
    // Mark that we're in a session
    settings_->setValue(CRASH_MARKER, true);
    settings_->sync();
}

QString SessionManager::get_settings_key(const QString& key) const {
    return QString("Preferences/%1/%2").arg(current_session_name_, key);
}

