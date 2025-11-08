/****************************************************************************
** Meta object code from reading C++ file 'upload_tab.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../include/gui/upload_tab.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'upload_tab.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ros2_dashboard__gui__UploadTab_t {
    QByteArrayData data[9];
    char stringdata0[147];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ros2_dashboard__gui__UploadTab_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ros2_dashboard__gui__UploadTab_t qt_meta_stringdata_ros2_dashboard__gui__UploadTab = {
    {
QT_MOC_LITERAL(0, 0, 30), // "ros2_dashboard::gui::UploadTab"
QT_MOC_LITERAL(1, 31, 22), // "on_queue_item_selected"
QT_MOC_LITERAL(2, 54, 0), // ""
QT_MOC_LITERAL(3, 55, 3), // "row"
QT_MOC_LITERAL(4, 59, 6), // "column"
QT_MOC_LITERAL(5, 66, 16), // "on_retry_clicked"
QT_MOC_LITERAL(6, 83, 17), // "on_cancel_clicked"
QT_MOC_LITERAL(7, 101, 26), // "on_clear_completed_clicked"
QT_MOC_LITERAL(8, 128, 18) // "on_refresh_clicked"

    },
    "ros2_dashboard::gui::UploadTab\0"
    "on_queue_item_selected\0\0row\0column\0"
    "on_retry_clicked\0on_cancel_clicked\0"
    "on_clear_completed_clicked\0"
    "on_refresh_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ros2_dashboard__gui__UploadTab[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    2,   39,    2, 0x08 /* Private */,
       5,    0,   44,    2, 0x08 /* Private */,
       6,    0,   45,    2, 0x08 /* Private */,
       7,    0,   46,    2, 0x08 /* Private */,
       8,    0,   47,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    3,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void ros2_dashboard::gui::UploadTab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<UploadTab *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->on_queue_item_selected((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 1: _t->on_retry_clicked(); break;
        case 2: _t->on_cancel_clicked(); break;
        case 3: _t->on_clear_completed_clicked(); break;
        case 4: _t->on_refresh_clicked(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ros2_dashboard::gui::UploadTab::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_ros2_dashboard__gui__UploadTab.data,
    qt_meta_data_ros2_dashboard__gui__UploadTab,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ros2_dashboard::gui::UploadTab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ros2_dashboard::gui::UploadTab::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ros2_dashboard__gui__UploadTab.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int ros2_dashboard::gui::UploadTab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
