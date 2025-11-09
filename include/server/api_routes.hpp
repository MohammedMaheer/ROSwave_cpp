/**
 * @file api_routes.hpp
 * @brief API route definitions for dashboard endpoints
 * @author Dashboard Team
 */

#pragma once

#include "rest_api_server.hpp"
#include <memory>

namespace ros2_dashboard::gui {
class MainWindow;  // Forward declaration
}

namespace ros2_dashboard::server {

/**
 * @class ApiRoutes
 * @brief Setup and manage all API endpoints
 * 
 * Defines routes for accessing:
 * - Topics and node information
 * - Metrics and statistics
 * - Recording management
 * - Configuration
 */
class ApiRoutes {
public:
    /**
     * @brief Setup all API routes on the server
     * @param server REST API server instance
     * @param main_window Pointer to main window (for data access)
     */
    static void setup_routes(RestApiServer& server, 
                           ros2_dashboard::gui::MainWindow* main_window);

private:
    // Topics API
    static HttpResponse get_topics(const HttpRequest& req,
                                  ros2_dashboard::gui::MainWindow* window);
    static HttpResponse get_topic_detail(const HttpRequest& req,
                                        ros2_dashboard::gui::MainWindow* window);
    static HttpResponse subscribe_topic(const HttpRequest& req,
                                       ros2_dashboard::gui::MainWindow* window);
    
    // Nodes API
    static HttpResponse get_nodes(const HttpRequest& req,
                                 ros2_dashboard::gui::MainWindow* window);
    static HttpResponse get_node_detail(const HttpRequest& req,
                                       ros2_dashboard::gui::MainWindow* window);
    
    // Metrics API
    static HttpResponse get_metrics(const HttpRequest& req,
                                   ros2_dashboard::gui::MainWindow* window);
    static HttpResponse get_metrics_history(const HttpRequest& req,
                                           ros2_dashboard::gui::MainWindow* window);
    
    // Services API
    static HttpResponse get_services(const HttpRequest& req,
                                    ros2_dashboard::gui::MainWindow* window);
    static HttpResponse call_service(const HttpRequest& req,
                                    ros2_dashboard::gui::MainWindow* window);
    
    // Status API
    static HttpResponse get_status(const HttpRequest& req,
                                  ros2_dashboard::gui::MainWindow* window);
    
    // Health API
    static HttpResponse get_health(const HttpRequest& req,
                                  ros2_dashboard::gui::MainWindow* window);
};

}  // namespace ros2_dashboard::server
