#pragma once

#include <QString>
#include <QStringList>
#include <QSettings>
#include <QTimer>
#include <QMainWindow>
#include <QByteArray>
#include <memory>

/**
 * @brief SessionManager handles auto-save and restore of dashboard state
 * 
 * Features:
 * - Auto-save selected topics
 * - Persist window geometry and layout
 * - Save user preferences
 * - Crash recovery
 * - Checkpoint system for critical data
 */
class SessionManager : public QObject {
    Q_OBJECT

public:
    static SessionManager& instance();

    /**
     * @brief Initialize session manager
     * @param main_window Main application window
     * @param auto_save_interval_ms Interval for auto-save (default: 30s)
     */
    void initialize(QMainWindow* main_window, int auto_save_interval_ms = 30000);

    /**
     * @brief Save current session state
     */
    void save_session();

    /**
     * @brief Restore previous session state
     */
    void restore_session();

    /**
     * @brief Save selected topics list
     */
    void save_selected_topics(const QStringList& topics);

    /**
     * @brief Get restored selected topics
     */
    QStringList get_restored_topics() const;

    /**
     * @brief Save window geometry and state
     */
    void save_window_state();

    /**
     * @brief Restore window geometry and state
     */
    void restore_window_state();

    /**
     * @brief Set a user preference
     */
    void set_preference(const QString& key, const QVariant& value);

    /**
     * @brief Get a user preference
     */
    QVariant get_preference(const QString& key, const QVariant& default_value = QVariant());

    /**
     * @brief Save recording settings
     */
    void save_recording_settings(const QString& output_dir, bool auto_compress);

    /**
     * @brief Get last recording directory
     */
    QString get_last_recording_dir() const;

    /**
     * @brief Create a checkpoint for recovery
     */
    void create_checkpoint(const QString& checkpoint_name, const QByteArray& data);

    /**
     * @brief Restore from checkpoint
     */
    QByteArray restore_checkpoint(const QString& checkpoint_name);

    /**
     * @brief Check if previous crash occurred
     */
    bool has_crash_recovery_data() const;

    /**
     * @brief Get crash recovery data
     */
    QByteArray get_crash_recovery_data();

    /**
     * @brief Mark session as clean (no crash)
     */
    void mark_session_clean();

    /**
     * @brief Enable auto-save
     */
    void enable_auto_save(bool enabled);

    /**
     * @brief Set auto-save interval
     */
    void set_auto_save_interval(int ms);

    /**
     * @brief Get current session name
     */
    QString get_current_session_name() const;

    /**
     * @brief Switch to named session
     */
    void switch_session(const QString& session_name);

    /**
     * @brief List all available sessions
     */
    QStringList list_sessions() const;

    /**
     * @brief Delete a session
     */
    void delete_session(const QString& session_name);

    /**
     * @brief Export session as file
     */
    bool export_session(const QString& file_path);

    /**
     * @brief Import session from file
     */
    bool import_session(const QString& file_path);

signals:
    /**
     * @brief Emitted when session is saved
     */
    void session_saved();

    /**
     * @brief Emitted when session is restored
     */
    void session_restored();

    /**
     * @brief Emitted before auto-save
     */
    void before_auto_save();

    /**
     * @brief Emitted after auto-save
     */
    void after_auto_save();

private:
    SessionManager();

    void on_auto_save_timeout();
    void setup_crash_detection();
    QString get_settings_key(const QString& key) const;

    QMainWindow* main_window_;
    std::unique_ptr<QSettings> settings_;
    std::unique_ptr<QTimer> auto_save_timer_;
    QString current_session_name_;
    bool auto_save_enabled_;
    static constexpr const char* CRASH_MARKER = "has_crash_marker";
    static constexpr const char* SESSION_GROUP = "Sessions";
};

