/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../include/gui/main_window.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ros2_dashboard__gui__MainWindow_t {
    QByteArrayData data[15];
    char stringdata0[230];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ros2_dashboard__gui__MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ros2_dashboard__gui__MainWindow_t qt_meta_stringdata_ros2_dashboard__gui__MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 31), // "ros2_dashboard::gui::MainWindow"
QT_MOC_LITERAL(1, 32, 14), // "on_refresh_all"
QT_MOC_LITERAL(2, 47, 0), // ""
QT_MOC_LITERAL(3, 48, 19), // "on_settings_changed"
QT_MOC_LITERAL(4, 68, 8), // "on_about"
QT_MOC_LITERAL(5, 77, 7), // "on_exit"
QT_MOC_LITERAL(6, 85, 13), // "on_export_all"
QT_MOC_LITERAL(7, 99, 16), // "on_health_report"
QT_MOC_LITERAL(8, 116, 14), // "on_diagnostics"
QT_MOC_LITERAL(9, 131, 14), // "on_clear_cache"
QT_MOC_LITERAL(10, 146, 18), // "on_maximize_toggle"
QT_MOC_LITERAL(11, 165, 21), // "on_fast_timer_timeout"
QT_MOC_LITERAL(12, 187, 21), // "on_slow_timer_timeout"
QT_MOC_LITERAL(13, 209, 14), // "on_tab_changed"
QT_MOC_LITERAL(14, 224, 5) // "index"

    },
    "ros2_dashboard::gui::MainWindow\0"
    "on_refresh_all\0\0on_settings_changed\0"
    "on_about\0on_exit\0on_export_all\0"
    "on_health_report\0on_diagnostics\0"
    "on_clear_cache\0on_maximize_toggle\0"
    "on_fast_timer_timeout\0on_slow_timer_timeout\0"
    "on_tab_changed\0index"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ros2_dashboard__gui__MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x08 /* Private */,
       3,    0,   75,    2, 0x08 /* Private */,
       4,    0,   76,    2, 0x08 /* Private */,
       5,    0,   77,    2, 0x08 /* Private */,
       6,    0,   78,    2, 0x08 /* Private */,
       7,    0,   79,    2, 0x08 /* Private */,
       8,    0,   80,    2, 0x08 /* Private */,
       9,    0,   81,    2, 0x08 /* Private */,
      10,    0,   82,    2, 0x08 /* Private */,
      11,    0,   83,    2, 0x08 /* Private */,
      12,    0,   84,    2, 0x08 /* Private */,
      13,    1,   85,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   14,

       0        // eod
};

void ros2_dashboard::gui::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->on_refresh_all(); break;
        case 1: _t->on_settings_changed(); break;
        case 2: _t->on_about(); break;
        case 3: _t->on_exit(); break;
        case 4: _t->on_export_all(); break;
        case 5: _t->on_health_report(); break;
        case 6: _t->on_diagnostics(); break;
        case 7: _t->on_clear_cache(); break;
        case 8: _t->on_maximize_toggle(); break;
        case 9: _t->on_fast_timer_timeout(); break;
        case 10: _t->on_slow_timer_timeout(); break;
        case 11: _t->on_tab_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ros2_dashboard::gui::MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_ros2_dashboard__gui__MainWindow.data,
    qt_meta_data_ros2_dashboard__gui__MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ros2_dashboard::gui::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ros2_dashboard::gui::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ros2_dashboard__gui__MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int ros2_dashboard::gui::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
