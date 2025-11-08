/****************************************************************************
** Meta object code from reading C++ file 'topics_tab.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../include/gui/topics_tab.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'topics_tab.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_ros2_dashboard__gui__TopicsTab_t {
    QByteArrayData data[13];
    char stringdata0[225];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ros2_dashboard__gui__TopicsTab_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ros2_dashboard__gui__TopicsTab_t qt_meta_stringdata_ros2_dashboard__gui__TopicsTab = {
    {
QT_MOC_LITERAL(0, 0, 30), // "ros2_dashboard::gui::TopicsTab"
QT_MOC_LITERAL(1, 31, 17), // "on_topics_updated"
QT_MOC_LITERAL(2, 49, 0), // ""
QT_MOC_LITERAL(3, 50, 38), // "std::vector<ros2_dashboard::T..."
QT_MOC_LITERAL(4, 89, 6), // "topics"
QT_MOC_LITERAL(5, 96, 18), // "on_message_updated"
QT_MOC_LITERAL(6, 115, 7), // "message"
QT_MOC_LITERAL(7, 123, 17), // "on_topic_selected"
QT_MOC_LITERAL(8, 141, 3), // "row"
QT_MOC_LITERAL(9, 145, 6), // "column"
QT_MOC_LITERAL(10, 152, 25), // "on_refresh_button_clicked"
QT_MOC_LITERAL(11, 178, 22), // "on_topics_updated_slot"
QT_MOC_LITERAL(12, 201, 23) // "on_message_updated_slot"

    },
    "ros2_dashboard::gui::TopicsTab\0"
    "on_topics_updated\0\0"
    "std::vector<ros2_dashboard::TopicInfo>\0"
    "topics\0on_message_updated\0message\0"
    "on_topic_selected\0row\0column\0"
    "on_refresh_button_clicked\0"
    "on_topics_updated_slot\0on_message_updated_slot"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ros2_dashboard__gui__TopicsTab[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x06 /* Public */,
       5,    1,   47,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    2,   50,    2, 0x08 /* Private */,
      10,    0,   55,    2, 0x08 /* Private */,
      11,    1,   56,    2, 0x08 /* Private */,
      12,    1,   59,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QString,    6,

 // slots: parameters
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    8,    9,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, QMetaType::QString,    6,

       0        // eod
};

void ros2_dashboard::gui::TopicsTab::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<TopicsTab *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->on_topics_updated((*reinterpret_cast< const std::vector<ros2_dashboard::TopicInfo>(*)>(_a[1]))); break;
        case 1: _t->on_message_updated((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->on_topic_selected((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: _t->on_refresh_button_clicked(); break;
        case 4: _t->on_topics_updated_slot((*reinterpret_cast< const std::vector<ros2_dashboard::TopicInfo>(*)>(_a[1]))); break;
        case 5: _t->on_message_updated_slot((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (TopicsTab::*)(const std::vector<ros2_dashboard::TopicInfo> & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&TopicsTab::on_topics_updated)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (TopicsTab::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&TopicsTab::on_message_updated)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject ros2_dashboard::gui::TopicsTab::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_ros2_dashboard__gui__TopicsTab.data,
    qt_meta_data_ros2_dashboard__gui__TopicsTab,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *ros2_dashboard::gui::TopicsTab::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ros2_dashboard::gui::TopicsTab::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_ros2_dashboard__gui__TopicsTab.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int ros2_dashboard::gui::TopicsTab::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void ros2_dashboard::gui::TopicsTab::on_topics_updated(const std::vector<ros2_dashboard::TopicInfo> & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void ros2_dashboard::gui::TopicsTab::on_message_updated(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
