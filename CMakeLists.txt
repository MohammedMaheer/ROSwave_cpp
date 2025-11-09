cmake_minimum_required(VERSION 3.16)
project(ros2_dashboard VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Add cmake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find required packages
find_package(Qt5 COMPONENTS Core Gui Widgets Concurrent REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(CURL REQUIRED)
find_package(LibArchive REQUIRED)

# QCustomPlot - optional, but recommended for charting
# Install: sudo apt-get install libqcustomplot-dev
# Or from source: https://www.qcustomplot.com/
find_package(QCustomPlot QUIET)
if(QCustomPlot_FOUND)
    message(STATUS "QCustomPlot found - enabling chart features")
else()
    message(WARNING "QCustomPlot not found - charts will be disabled. Install with: sudo apt-get install libqcustomplot-dev")
endif()

# SQLite3 - handle as system library
find_library(SQLITE3_LIBRARY sqlite3)
if (NOT SQLITE3_LIBRARY)
    message(FATAL_ERROR "sqlite3 library not found")
endif()
find_path(SQLITE3_INCLUDE_DIR sqlite3.h)
if (NOT SQLITE3_INCLUDE_DIR)
    message(FATAL_ERROR "sqlite3 header not found")
endif()
include_directories(${SQLITE3_INCLUDE_DIR})

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Source files
set(SOURCES
    src/main.cpp
    src/ros2_manager.cpp
    src/metrics_collector.cpp
    src/metrics_history_buffer.cpp
    src/network_manager.cpp
    src/ml_exporter.cpp
    src/async_worker.cpp
    src/topic_monitor.cpp
    src/process_guard.cpp
    src/alert_manager.cpp
    src/session_manager.cpp
    src/ros2_metrics_collector.cpp
    src/gui/main_window.cpp
    src/gui/topics_tab.cpp
    src/gui/nodes_tab.cpp
    src/gui/selected_topics_tab.cpp
    src/gui/services_tab.cpp
    src/gui/recording_tab.cpp
    src/gui/metrics_tab.cpp
    src/gui/export_tab.cpp
    src/gui/network_tab.cpp
    src/gui/upload_tab.cpp
    src/gui/settings_dialog.cpp
    src/gui/message_inspector_tab.cpp
    src/gui/advanced_chart_widget.cpp
    src/gui/advanced_filter.cpp
    src/gui/topic_dependency_graph.cpp
    src/gui/context_menus.cpp
    src/logging.cpp
)

set(HEADERS
    include/ros2_manager.hpp
    include/metrics_collector.hpp
    include/metrics_history_buffer.hpp
    include/network_manager.hpp
    include/ml_exporter.hpp
    include/async_worker.hpp
    include/topic_monitor.hpp
    include/alert_manager.hpp
    include/session_manager.hpp
    include/ros2_metrics_collector.hpp
    include/gui/main_window.hpp
    include/gui/topics_tab.hpp
    include/gui/nodes_tab.hpp
    include/gui/selected_topics_tab.hpp
    include/gui/services_tab.hpp
    include/gui/recording_tab.hpp
    include/gui/metrics_tab.hpp
    include/gui/export_tab.hpp
    include/gui/network_tab.hpp
    include/gui/upload_tab.hpp
    include/gui/settings_dialog.hpp
    include/gui/message_inspector_tab.hpp
    include/gui/advanced_chart_widget.hpp
    include/gui/advanced_filter.hpp
    include/gui/topic_dependency_graph.hpp
    include/gui/context_menus.hpp
    include/logging.hpp
    include/adaptive_cache_layer.hpp
    include/performance_optimizations.hpp
    include/gui/ui_improvements.hpp
)

# Main executable
add_executable(ros2_dashboard ${SOURCES} ${HEADERS})

target_compile_options(ros2_dashboard PRIVATE -O3 -march=native -Wall -Wextra)

target_link_libraries(ros2_dashboard
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::Concurrent
    rclcpp::rclcpp
    rosbag2_cpp::rosbag2_cpp
    ${SQLITE3_LIBRARY}
    nlohmann_json::nlohmann_json
    CURL::libcurl
    LibArchive::LibArchive
)

# Link QCustomPlot if available
if(QCustomPlot_FOUND)
    target_link_libraries(ros2_dashboard QCustomPlot::QCustomPlot)
    target_compile_definitions(ros2_dashboard PRIVATE HAVE_QCUSTOMPLOT=1)
endif()

# Upload server executable
add_executable(ros2_upload_server src/server/upload_server.cpp)
target_link_libraries(ros2_upload_server
    CURL::libcurl
    ${SQLITE3_LIBRARY}
    nlohmann_json::nlohmann_json
    LibArchive::LibArchive
)

# Performance verification tool
add_executable(ros2_verify_performance src/tools/verify_performance.cpp)
target_link_libraries(ros2_verify_performance
    rclcpp::rclcpp
    nlohmann_json::nlohmann_json
)

# ===== ASAN/UBSAN Build Targets =====
# Build with AddressSanitizer: cmake -DCMAKE_BUILD_TYPE=ASAN ..
set(CMAKE_CXX_FLAGS_ASAN "-g -O1 -fsanitize=address,leak -fno-omit-frame-pointer -fno-optimize-sibling-calls" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_ASAN "-g -O1 -fsanitize=address,leak -fno-omit-frame-pointer -fno-optimize-sibling-calls" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_ASAN "-fsanitize=address,leak" CACHE STRING "" FORCE)
set(CMAKE_SHARED_LINKER_FLAGS_ASAN "-fsanitize=address,leak" CACHE STRING "" FORCE)

# Build with UndefinedBehaviorSanitizer: cmake -DCMAKE_BUILD_TYPE=UBSAN ..
set(CMAKE_CXX_FLAGS_UBSAN "-g -O1 -fsanitize=undefined -fno-omit-frame-pointer -fno-sanitize-recover=all" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_UBSAN "-g -O1 -fsanitize=undefined -fno-omit-frame-pointer -fno-sanitize-recover=all" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_UBSAN "-fsanitize=undefined" CACHE STRING "" FORCE)
set(CMAKE_SHARED_LINKER_FLAGS_UBSAN "-fsanitize=undefined" CACHE STRING "" FORCE)

# Build with both ASAN and UBSAN: cmake -DCMAKE_BUILD_TYPE=ASAN_UBSAN ..
set(CMAKE_CXX_FLAGS_ASAN_UBSAN "-g -O1 -fsanitize=address,leak,undefined -fno-omit-frame-pointer -fno-optimize-sibling-calls -fno-sanitize-recover=all" CACHE STRING "" FORCE)
set(CMAKE_C_FLAGS_ASAN_UBSAN "-g -O1 -fsanitize=address,leak,undefined -fno-omit-frame-pointer -fno-optimize-sibling-calls -fno-sanitize-recover=all" CACHE STRING "" FORCE)
set(CMAKE_EXE_LINKER_FLAGS_ASAN_UBSAN "-fsanitize=address,leak,undefined" CACHE STRING "" FORCE)
set(CMAKE_SHARED_LINKER_FLAGS_ASAN_UBSAN "-fsanitize=address,leak,undefined" CACHE STRING "" FORCE)

message(STATUS "Available build types: Release, Debug, ASAN, UBSAN, ASAN_UBSAN")

# ===== Unit Testing =====
enable_testing()
find_package(GTest QUIET)

if(GTest_FOUND)
    message(STATUS "GTest found - enabling unit tests")
    
    add_executable(test_containers tests/test_containers.cpp src/metrics_history_buffer.cpp)
    
    target_link_libraries(test_containers
        GTest::GTest
        GTest::Main
        Qt5::Core
        ${SQLITE3_LIBRARY}
        nlohmann_json::nlohmann_json
    )
    
    target_compile_options(test_containers PRIVATE -O2 -Wall -Wextra)
    
    add_test(NAME ContainersTest COMMAND test_containers)
    
else()
    message(WARNING "GTest not found - unit tests disabled. Install with: sudo apt-get install libgtest-dev")
endif()

# Standalone logging test (no GTest required)
add_executable(test_logging tests/test_logging.cpp)
target_link_libraries(test_logging
    nlohmann_json::nlohmann_json
)
target_compile_options(test_logging PRIVATE -O2 -Wall -Wextra)
add_test(NAME LoggingTest COMMAND test_logging)

# Standalone context menu test (no GUI required)
add_executable(test_context_menus_standalone tests/test_context_menus_standalone.cpp)
target_compile_options(test_context_menus_standalone PRIVATE -O2 -Wall -Wextra)
add_test(NAME ContextMenusStandaloneTest COMMAND test_context_menus_standalone)

# Installation rules
install(TARGETS ros2_dashboard ros2_upload_server ros2_verify_performance
    RUNTIME DESTINATION bin
)

install(FILES config/config.json DESTINATION etc/ros2_dashboard)
install(DIRECTORY docs/ DESTINATION share/doc/ros2_dashboard)
