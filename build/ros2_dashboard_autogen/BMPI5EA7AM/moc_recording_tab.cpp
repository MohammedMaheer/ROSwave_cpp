/****************************************************************************
** Meta object code from reading C++ file 'recording_tab.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../include/gui/recording_tab.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'recording_tab.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ros2_dashboard__gui__RecordingTab_t {
    QByteArrayData data[13];
    char stringdata0[261];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ros2_dashboard__gui__RecordingTab_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ros2_dashboard__gui__RecordingTab_t qt_meta_stringdata_ros2_dashboard__gui__RecordingTab = {
    {
QT_MOC_LITERAL(0, 0, 33), // "ros2_dashboard::gui::Recordin..."
QT_MOC_LITERAL(1, 34, 27), // "on_recording_status_updated"
QT_MOC_LITERAL(2, 62, 0), // ""
QT_MOC_LITERAL(3, 63, 12), // "is_recording"
QT_MOC_LITERAL(4, 76, 26), // "on_start_recording_clicked"
QT_MOC_LITERAL(5, 103, 25), // "on_stop_recording_clicked"
QT_MOC_LITERAL(6, 129, 27), // "on_browse_directory_clicked"
QT_MOC_LITERAL(7, 157, 23), // "on_refresh_bags_clicked"
QT_MOC_LITERAL(8, 181, 15), // "on_bag_selected"
QT_MOC_LITERAL(9, 197, 3), // "row"
QT_MOC_LITERAL(10, 201, 6), // "column"
QT_MOC_LITERAL(11, 208, 28), // "on_open_file_manager_clicked"
QT_MOC_LITERAL(12, 237, 23) // "on_play_in_rviz_clicked"

    },
    "ros2_dashboard::gui::RecordingTab\0"
    "on_recording_status_updated\0\0is_recording\0"
    "on_start_recording_clicked\0"
    "on_stop_recording_clicked\0"
    "on_browse_directory_clicked\0"
    "on_refresh_bags_clicked\0on_bag_selected\0"
    "row\0column\0on_open_file_manager_clicked\0"
    "on_play_in_rviz_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ros2_dashboard__gui__RecordingTab[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   54,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   57,    2, 0x08 /* Private */,
       5,    0,   58,    2, 0x08 /* Private */,
       6,    0,   59,    2, 0x08 /* Private */,
       7,    0,   60,    2, 0x08 /* Private */,
       8,    2,   61,    2, 0x08 /* Private */,
      11,    0,   66,    2, 0x08 /* Private */,
      12,    0,   67,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool,    3,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    9,   10,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void ros2_dashboard::gui::RecordingTab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<RecordingTab *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->on_recording_status_updated((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->on_start_recording_clicked(); break;
        case 2: _t->on_stop_recording_clicked(); break;
        case 3: _t->on_browse_directory_clicked(); break;
        case 4: _t->on_refresh_bags_clicked(); break;
        case 5: _t->on_bag_selected((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 6: _t->on_open_file_manager_clicked(); break;
        case 7: _t->on_play_in_rviz_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (RecordingTab::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&RecordingTab::on_recording_status_updated)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ros2_dashboard::gui::RecordingTab::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_ros2_dashboard__gui__RecordingTab.data,
    qt_meta_data_ros2_dashboard__gui__RecordingTab,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ros2_dashboard::gui::RecordingTab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ros2_dashboard::gui::RecordingTab::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ros2_dashboard__gui__RecordingTab.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int ros2_dashboard::gui::RecordingTab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void ros2_dashboard::gui::RecordingTab::on_recording_status_updated(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
