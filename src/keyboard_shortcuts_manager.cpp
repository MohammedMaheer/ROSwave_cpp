#include "../include/keyboard_shortcuts_manager.hpp"
#include <QSettings>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <algorithm>

KeyboardShortcutsManager::KeyboardShortcutsManager(QMainWindow* main_window)
    : QObject(main_window), main_window_(main_window) {
    setup_default_shortcuts();
    load_shortcuts_from_settings();
    initialize_shortcuts();
}

void KeyboardShortcutsManager::setup_default_shortcuts() {
    // View shortcuts
    shortcuts_[ShortcutAction::TOGGLE_SIDEBAR] = {
        ShortcutAction::TOGGLE_SIDEBAR, "Toggle Sidebar", "Show/hide left sidebar",
        ShortcutCategory::VIEW, Qt::CTRL + Qt::Key_B, Qt::CTRL + Qt::Key_B, true, true
    };
    
    shortcuts_[ShortcutAction::TOGGLE_FULLSCREEN] = {
        ShortcutAction::TOGGLE_FULLSCREEN, "Toggle Fullscreen", "Enter/exit fullscreen mode",
        ShortcutCategory::VIEW, Qt::Key_F11, Qt::Key_F11, true, true
    };
    
    shortcuts_[ShortcutAction::ZOOM_IN] = {
        ShortcutAction::ZOOM_IN, "Zoom In", "Increase chart zoom level",
        ShortcutCategory::VIEW, Qt::CTRL + Qt::Key_Plus, Qt::CTRL + Qt::Key_Plus, true, true
    };
    
    shortcuts_[ShortcutAction::ZOOM_OUT] = {
        ShortcutAction::ZOOM_OUT, "Zoom Out", "Decrease chart zoom level",
        ShortcutCategory::VIEW, Qt::CTRL + Qt::Key_Minus, Qt::CTRL + Qt::Key_Minus, true, true
    };
    
    shortcuts_[ShortcutAction::FIT_TO_WINDOW] = {
        ShortcutAction::FIT_TO_WINDOW, "Fit to Window", "Fit all data to current window",
        ShortcutCategory::VIEW, Qt::CTRL + Qt::Key_0, Qt::CTRL + Qt::Key_0, true, true
    };
    
    // Navigation shortcuts
    shortcuts_[ShortcutAction::NEXT_TAB] = {
        ShortcutAction::NEXT_TAB, "Next Tab", "Switch to next tab",
        ShortcutCategory::VIEW, Qt::CTRL + Qt::Key_Tab, Qt::CTRL + Qt::Key_Tab, true, true
    };
    
    shortcuts_[ShortcutAction::PREV_TAB] = {
        ShortcutAction::PREV_TAB, "Previous Tab", "Switch to previous tab",
        ShortcutCategory::VIEW, Qt::CTRL + Qt::SHIFT + Qt::Key_Tab, Qt::CTRL + Qt::SHIFT + Qt::Key_Tab, true, true
    };
    
    shortcuts_[ShortcutAction::FOCUS_SEARCH] = {
        ShortcutAction::FOCUS_SEARCH, "Focus Search", "Focus on search input",
        ShortcutCategory::VIEW, Qt::CTRL + Qt::Key_F, Qt::CTRL + Qt::Key_F, true, true
    };
    
    shortcuts_[ShortcutAction::OPEN_TOPICS_TAB] = {
        ShortcutAction::OPEN_TOPICS_TAB, "Open Topics Tab", "Switch to topics tab",
        ShortcutCategory::VIEW, Qt::CTRL + Qt::Key_T, Qt::CTRL + Qt::Key_T, true, true
    };
    
    // Control shortcuts
    shortcuts_[ShortcutAction::PLAY_PAUSE] = {
        ShortcutAction::PLAY_PAUSE, "Play/Pause", "Play or pause playback",
        ShortcutCategory::CONTROL, Qt::Key_Space, Qt::Key_Space, true, true
    };
    
    shortcuts_[ShortcutAction::STOP] = {
        ShortcutAction::STOP, "Stop", "Stop playback",
        ShortcutCategory::CONTROL, Qt::CTRL + Qt::Key_X, Qt::CTRL + Qt::Key_X, true, true
    };
    
    shortcuts_[ShortcutAction::STEP_FORWARD] = {
        ShortcutAction::STEP_FORWARD, "Step Forward", "Step forward one frame",
        ShortcutCategory::CONTROL, Qt::Key_Right, Qt::Key_Right, true, true
    };
    
    shortcuts_[ShortcutAction::JUMP_START] = {
        ShortcutAction::JUMP_START, "Jump to Start", "Jump to beginning",
        ShortcutCategory::CONTROL, Qt::Key_Home, Qt::Key_Home, true, true
    };
    
    shortcuts_[ShortcutAction::JUMP_END] = {
        ShortcutAction::JUMP_END, "Jump to End", "Jump to end",
        ShortcutCategory::CONTROL, Qt::Key_End, Qt::Key_End, true, true
    };
    
    // Topic management shortcuts
    shortcuts_[ShortcutAction::SELECT_ALL_TOPICS] = {
        ShortcutAction::SELECT_ALL_TOPICS, "Select All Topics", "Select all topics",
        ShortcutCategory::TOPIC, Qt::CTRL + Qt::Key_A, Qt::CTRL + Qt::Key_A, true, true
    };
    
    shortcuts_[ShortcutAction::DESELECT_ALL_TOPICS] = {
        ShortcutAction::DESELECT_ALL_TOPICS, "Deselect All Topics", "Deselect all topics",
        ShortcutCategory::TOPIC, Qt::CTRL + Qt::SHIFT + Qt::Key_A, Qt::CTRL + Qt::SHIFT + Qt::Key_A, true, true
    };
    
    shortcuts_[ShortcutAction::DELETE_TOPIC] = {
        ShortcutAction::DELETE_TOPIC, "Delete Topic", "Remove selected topic",
        ShortcutCategory::TOPIC, Qt::Key_Delete, Qt::Key_Delete, true, true
    };
    
    shortcuts_[ShortcutAction::FIND_TOPIC] = {
        ShortcutAction::FIND_TOPIC, "Find Topic", "Open topic search dialog",
        ShortcutCategory::TOPIC, Qt::CTRL + Qt::Key_F, Qt::CTRL + Qt::Key_F, true, true
    };
    
    // Recording shortcuts
    shortcuts_[ShortcutAction::START_RECORDING] = {
        ShortcutAction::START_RECORDING, "Start Recording", "Start recording topics",
        ShortcutCategory::RECORDING, Qt::CTRL + Qt::Key_R, Qt::CTRL + Qt::Key_R, true, true
    };
    
    shortcuts_[ShortcutAction::STOP_RECORDING] = {
        ShortcutAction::STOP_RECORDING, "Stop Recording", "Stop recording topics",
        ShortcutCategory::RECORDING, Qt::CTRL + Qt::SHIFT + Qt::Key_R, Qt::CTRL + Qt::SHIFT + Qt::Key_R, true, true
    };
    
    shortcuts_[ShortcutAction::EXPORT_RECORDING] = {
        ShortcutAction::EXPORT_RECORDING, "Export Recording", "Export recorded data",
        ShortcutCategory::RECORDING, Qt::CTRL + Qt::SHIFT + Qt::Key_E, Qt::CTRL + Qt::SHIFT + Qt::Key_E, true, true
    };
    
    // Export shortcuts
    shortcuts_[ShortcutAction::EXPORT_AS_CSV] = {
        ShortcutAction::EXPORT_AS_CSV, "Export as CSV", "Export data as CSV",
        ShortcutCategory::EXPORT, Qt::CTRL + Qt::Key_E, Qt::CTRL + Qt::Key_E, true, true
    };
    
    shortcuts_[ShortcutAction::EXPORT_AS_JSON] = {
        ShortcutAction::EXPORT_AS_JSON, "Export as JSON", "Export data as JSON",
        ShortcutCategory::EXPORT, Qt::CTRL + Qt::ALT + Qt::Key_E, Qt::CTRL + Qt::ALT + Qt::Key_E, true, true
    };
    
    shortcuts_[ShortcutAction::COPY_SELECTION] = {
        ShortcutAction::COPY_SELECTION, "Copy Selection", "Copy selected data",
        ShortcutCategory::EXPORT, Qt::CTRL + Qt::Key_C, Qt::CTRL + Qt::Key_C, true, false
    };
    
    shortcuts_[ShortcutAction::SELECT_ALL] = {
        ShortcutAction::SELECT_ALL, "Select All", "Select all data",
        ShortcutCategory::EXPORT, Qt::CTRL + Qt::Key_A, Qt::CTRL + Qt::Key_A, true, false
    };
    
    // Settings shortcuts
    shortcuts_[ShortcutAction::OPEN_PREFERENCES] = {
        ShortcutAction::OPEN_PREFERENCES, "Preferences", "Open preferences dialog",
        ShortcutCategory::SETTINGS, Qt::CTRL + Qt::Key_Comma, Qt::CTRL + Qt::Key_Comma, true, true
    };
    
    shortcuts_[ShortcutAction::OPEN_SETTINGS] = {
        ShortcutAction::OPEN_SETTINGS, "Settings", "Open settings dialog",
        ShortcutCategory::SETTINGS, Qt::CTRL + Qt::ALT + Qt::Key_S, Qt::CTRL + Qt::ALT + Qt::Key_S, true, true
    };
    
    shortcuts_[ShortcutAction::RELOAD_CONFIG] = {
        ShortcutAction::RELOAD_CONFIG, "Reload Config", "Reload configuration",
        ShortcutCategory::SETTINGS, Qt::CTRL + Qt::SHIFT + Qt::Key_L, Qt::CTRL + Qt::SHIFT + Qt::Key_L, true, true
    };
    
    // Help shortcuts
    shortcuts_[ShortcutAction::OPEN_HELP] = {
        ShortcutAction::OPEN_HELP, "Help", "Open help documentation",
        ShortcutCategory::HELP, Qt::Key_F1, Qt::Key_F1, true, true
    };
    
    shortcuts_[ShortcutAction::SHOW_SHORTCUTS] = {
        ShortcutAction::SHOW_SHORTCUTS, "Show Keyboard Shortcuts", "Display all keyboard shortcuts",
        ShortcutCategory::HELP, Qt::CTRL + Qt::Key_Question, Qt::CTRL + Qt::Key_Question, true, true
    };
    
    shortcuts_[ShortcutAction::SHOW_ABOUT] = {
        ShortcutAction::SHOW_ABOUT, "About", "Show about dialog",
        ShortcutCategory::HELP, Qt::ALT + Qt::Key_F1, Qt::ALT + Qt::Key_F1, true, true
    };
    
    // Debug shortcuts
    shortcuts_[ShortcutAction::SHOW_CONSOLE] = {
        ShortcutAction::SHOW_CONSOLE, "Show Console", "Show debug console",
        ShortcutCategory::SETTINGS, Qt::CTRL + Qt::ALT + Qt::Key_C, Qt::CTRL + Qt::ALT + Qt::Key_C, true, true
    };
    
    shortcuts_[ShortcutAction::RELOAD_PLUGINS] = {
        ShortcutAction::RELOAD_PLUGINS, "Reload Plugins", "Reload plugin system",
        ShortcutCategory::SETTINGS, Qt::CTRL + Qt::ALT + Qt::Key_P, Qt::CTRL + Qt::ALT + Qt::Key_P, true, true
    };
    
    // Build reverse lookup map
    for (auto it = shortcuts_.begin(); it != shortcuts_.end(); ++it) {
        action_name_map_[it.value().name] = it.key();
    }
}

void KeyboardShortcutsManager::initialize_shortcuts() {
    connect_shortcuts();
}

void KeyboardShortcutsManager::register_shortcut(ShortcutAction action, std::function<void()> handler) {
    handlers_[action] = handler;
}

bool KeyboardShortcutsManager::customize_shortcut(ShortcutAction action, const QKeySequence& new_sequence) {
    auto it = shortcuts_.find(action);
    if (it == shortcuts_.end() || !it.value().is_customizable) {
        return false;
    }
    
    // Check for conflicts
    if (is_sequence_used(new_sequence, action)) {
        return false;
    }
    
    if (!validate_sequence(new_sequence)) {
        return false;
    }
    
    it.value().current_sequence = new_sequence;
    
    // Update the QShortcut object if it exists
    if (shortcut_objects_.contains(action)) {
        shortcut_objects_[action]->setKey(new_sequence);
    }
    
    emit shortcut_customized(action, new_sequence);
    return true;
}

void KeyboardShortcutsManager::reset_shortcut_to_default(ShortcutAction action) {
    auto it = shortcuts_.find(action);
    if (it != shortcuts_.end()) {
        it.value().current_sequence = it.value().default_sequence;
        if (shortcut_objects_.contains(action)) {
            shortcut_objects_[action]->setKey(it.value().default_sequence);
        }
    }
}

void KeyboardShortcutsManager::reset_all_shortcuts() {
    for (auto it = shortcuts_.begin(); it != shortcuts_.end(); ++it) {
        reset_shortcut_to_default(it.key());
    }
    emit shortcuts_reset();
}

KeyboardShortcutsManager::ShortcutDef KeyboardShortcutsManager::get_shortcut_definition(ShortcutAction action) const {
    auto it = shortcuts_.find(action);
    if (it != shortcuts_.end()) {
        return it.value();
    }
    return ShortcutDef{};
}

QMap<KeyboardShortcutsManager::ShortcutAction, KeyboardShortcutsManager::ShortcutDef>
KeyboardShortcutsManager::get_all_shortcuts() const {
    return shortcuts_;
}

QList<KeyboardShortcutsManager::ShortcutDef>
KeyboardShortcutsManager::get_shortcuts_by_category(ShortcutCategory category) const {
    QList<ShortcutDef> result;
    for (const auto& shortcut : shortcuts_) {
        if (shortcut.category == category) {
            result.append(shortcut);
        }
    }
    return result;
}

void KeyboardShortcutsManager::set_shortcut_enabled(ShortcutAction action, bool enabled) {
    auto it = shortcuts_.find(action);
    if (it != shortcuts_.end()) {
        it.value().enabled = enabled;
        if (shortcut_objects_.contains(action)) {
            shortcut_objects_[action]->setEnabled(enabled);
        }
    }
}

bool KeyboardShortcutsManager::is_shortcut_enabled(ShortcutAction action) const {
    auto it = shortcuts_.find(action);
    if (it != shortcuts_.end()) {
        return it.value().enabled;
    }
    return false;
}

void KeyboardShortcutsManager::load_shortcuts_from_settings() {
    QSettings settings("ROSwave", "ROSwave");
    settings.beginGroup("Shortcuts");
    
    for (auto it = shortcuts_.begin(); it != shortcuts_.end(); ++it) {
        QString key_name = it.value().name.replace(" ", "_");
        QString saved_sequence = settings.value(key_name, it.value().default_sequence.toString()).toString();
        it.value().current_sequence = QKeySequence(saved_sequence);
    }
    
    settings.endGroup();
}

void KeyboardShortcutsManager::save_shortcuts_to_settings() const {
    QSettings settings("ROSwave", "ROSwave");
    settings.beginGroup("Shortcuts");
    
    for (const auto& shortcut : shortcuts_) {
        QString key_name = shortcut.name.replace(" ", "_");
        settings.setValue(key_name, shortcut.current_sequence.toString());
    }
    
    settings.endGroup();
    settings.sync();
}

QList<QPair<KeyboardShortcutsManager::ShortcutAction, KeyboardShortcutsManager::ShortcutAction>>
KeyboardShortcutsManager::find_conflicts() const {
    QList<QPair<ShortcutAction, ShortcutAction>> conflicts;
    
    for (auto it1 = shortcuts_.begin(); it1 != shortcuts_.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != shortcuts_.end(); ++it2) {
            if (it1.value().current_sequence == it2.value().current_sequence &&
                !it1.value().current_sequence.isEmpty() &&
                it1.value().enabled && it2.value().enabled) {
                conflicts.append({it1.key(), it2.key()});
            }
        }
    }
    
    return conflicts;
}

QList<KeyboardShortcutsManager::ShortcutAction>
KeyboardShortcutsManager::get_actions_for_sequence(const QKeySequence& sequence) const {
    QList<ShortcutAction> actions;
    for (auto it = shortcuts_.begin(); it != shortcuts_.end(); ++it) {
        if (it.value().current_sequence == sequence && it.value().enabled) {
            actions.append(it.key());
        }
    }
    return actions;
}

bool KeyboardShortcutsManager::is_sequence_used(const QKeySequence& sequence, ShortcutAction except_action) const {
    for (auto it = shortcuts_.begin(); it != shortcuts_.end(); ++it) {
        if (it.key() != except_action && it.value().current_sequence == sequence && it.value().enabled) {
            return true;
        }
    }
    return false;
}

QString KeyboardShortcutsManager::export_shortcuts_as_json() const {
    QJsonObject root;
    QJsonArray shortcuts_array;
    
    for (const auto& shortcut : shortcuts_) {
        QJsonObject shortcut_obj;
        shortcut_obj["name"] = shortcut.name;
        shortcut_obj["action"] = static_cast<int>(shortcut.action);
        shortcut_obj["sequence"] = shortcut.current_sequence.toString();
        shortcut_obj["enabled"] = shortcut.enabled;
        shortcuts_array.append(shortcut_obj);
    }
    
    root["shortcuts"] = shortcuts_array;
    QJsonDocument doc(root);
    return QString::fromUtf8(doc.toJson());
}

bool KeyboardShortcutsManager::import_shortcuts_from_json(const QString& json_data) {
    QJsonDocument doc = QJsonDocument::fromJson(json_data.toUtf8());
    if (!doc.isObject()) {
        return false;
    }
    
    QJsonObject root = doc.object();
    QJsonArray shortcuts_array = root["shortcuts"].toArray();
    
    for (const auto& item : shortcuts_array) {
        QJsonObject obj = item.toObject();
        QString name = obj["name"].toString();
        QKeySequence sequence(obj["sequence"].toString());
        
        auto it = action_name_map_.find(name);
        if (it != action_name_map_.end()) {
            customize_shortcut(it.value(), sequence);
        }
    }
    
    return true;
}

void KeyboardShortcutsManager::connect_shortcuts() {
    for (auto it = shortcuts_.begin(); it != shortcuts_.end(); ++it) {
        if (it.value().enabled && !it.value().current_sequence.isEmpty()) {
            QShortcut* shortcut = new QShortcut(it.value().current_sequence, main_window_);
            shortcut->setContext(Qt::ApplicationShortcut);
            
            connect(shortcut, &QShortcut::activated, this, [this, action = it.key()]() {
                on_shortcut_triggered(action);
            });
            
            shortcut_objects_[it.key()] = shortcut;
        }
    }
}

void KeyboardShortcutsManager::on_shortcut_triggered(ShortcutAction action) {
    emit shortcut_triggered(action);
    
    if (handlers_.contains(action)) {
        handlers_[action]();
    }
}

bool KeyboardShortcutsManager::validate_sequence(const QKeySequence& sequence) const {
    // Reject empty sequences
    if (sequence.isEmpty()) {
        return false;
    }
    
    // Reject sequences that are too long (more than 4 keys)
    if (sequence.count() > 4) {
        return false;
    }
    
    return true;
}
