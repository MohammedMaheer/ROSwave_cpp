/**
 * @file main.cpp
 * @brief Application entry point
 */

#include <QApplication>
#include <iostream>
#include "gui/main_window.hpp"
#include "ros2_manager.hpp"

int main(int argc, char* argv[]) {
    try {
        QApplication app(argc, argv);

        // Register custom types for signal/slot communication across threads
        qRegisterMetaType<std::vector<ros2_dashboard::TopicInfo>>("std::vector<ros2_dashboard::TopicInfo>");
        qRegisterMetaType<std::vector<ros2_dashboard::NodeInfo>>("std::vector<ros2_dashboard::NodeInfo>");
        qRegisterMetaType<std::vector<ros2_dashboard::ServiceInfo>>("std::vector<ros2_dashboard::ServiceInfo>");

        // Set application metadata
        QApplication::setApplicationName("ROS2 Dashboard");
        QApplication::setApplicationVersion("1.0.0");

        std::cout << "[DASHBOARD] Starting ROS2 Dashboard v1.0.0..." << std::endl;

        // Create and show main window
        ros2_dashboard::gui::MainWindow window;
        
        if (!window.initialize()) {
            std::cerr << "[DASHBOARD] Failed to initialize application" << std::endl;
            return 1;
        }

        std::cout << "[DASHBOARD] Application initialized successfully" << std::endl;
        window.show();
        std::cout << "[DASHBOARD] Window shown, entering event loop..." << std::endl;
        return app.exec();

    } catch (const std::exception& e) {
        std::cerr << "[DASHBOARD] Fatal error: " << e.what() << std::endl;
        return 1;
    }
}
