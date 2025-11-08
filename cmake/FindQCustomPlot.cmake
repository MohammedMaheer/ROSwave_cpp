# Find module for QCustomPlot library
# Defines: QCustomPlot_FOUND, QCustomPlot_INCLUDE_DIR, QCustomPlot_LIBRARY
#          QCustomPlot::QCustomPlot (IMPORTED target)

find_path(QCustomPlot_INCLUDE_DIR qcustomplot.h
    HINTS
        /usr/include
        /usr/local/include
        /opt/local/include
        ${QT_INCLUDE_DIR}
    PATH_SUFFIXES qcustomplot
)

find_library(QCustomPlot_LIBRARY
    NAMES qcustomplot libqcustomplot
    HINTS
        /usr/lib
        /usr/local/lib
        /opt/local/lib
    PATH_SUFFIXES x86_64-linux-gnu lib64 lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(QCustomPlot
    REQUIRED_VARS QCustomPlot_LIBRARY QCustomPlot_INCLUDE_DIR
)

if(QCustomPlot_FOUND AND NOT TARGET QCustomPlot::QCustomPlot)
    add_library(QCustomPlot::QCustomPlot SHARED IMPORTED)
    set_target_properties(QCustomPlot::QCustomPlot PROPERTIES
        IMPORTED_LOCATION "${QCustomPlot_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${QCustomPlot_INCLUDE_DIR}"
    )
endif()

mark_as_advanced(QCustomPlot_INCLUDE_DIR QCustomPlot_LIBRARY)
