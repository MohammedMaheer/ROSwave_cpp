/**
 * @file rest_api_server.hpp
 * @brief REST API server for dashboard data access
 * @author Dashboard Team
 */

#pragma once

#include <string>
#include <memory>
#include <map>
#include <vector>
#include <functional>
#include <nlohmann/json.hpp>

namespace ros2_dashboard::server {

using json = nlohmann::json;

/**
 * @struct HttpRequest
 * @brief HTTP request information
 */
struct HttpRequest {
    std::string method;           // GET, POST, PUT, DELETE, etc.
    std::string path;             // Request path
    std::map<std::string, std::string> headers;
    std::string body;             // Request body
    std::map<std::string, std::string> query_params;
};

/**
 * @struct HttpResponse
 * @brief HTTP response information
 */
struct HttpResponse {
    int status_code = 200;
    std::map<std::string, std::string> headers;
    std::string body;
    
    // Common status codes
    static HttpResponse ok(const json& data) {
        HttpResponse r;
        r.status_code = 200;
        r.body = data.dump();
        r.headers["Content-Type"] = "application/json";
        return r;
    }
    
    static HttpResponse created(const json& data) {
        HttpResponse r;
        r.status_code = 201;
        r.body = data.dump();
        r.headers["Content-Type"] = "application/json";
        return r;
    }
    
    static HttpResponse bad_request(const std::string& message) {
        HttpResponse r;
        r.status_code = 400;
        json error;
        error["error"] = message;
        r.body = error.dump();
        r.headers["Content-Type"] = "application/json";
        return r;
    }
    
    static HttpResponse not_found() {
        HttpResponse r;
        r.status_code = 404;
        json error;
        error["error"] = "Not Found";
        r.body = error.dump();
        r.headers["Content-Type"] = "application/json";
        return r;
    }
    
    static HttpResponse internal_error(const std::string& message) {
        HttpResponse r;
        r.status_code = 500;
        json error;
        error["error"] = message;
        r.body = error.dump();
        r.headers["Content-Type"] = "application/json";
        return r;
    }
};

/**
 * @typedef RouteHandler
 * @brief Handler function type for HTTP routes
 */
typedef std::function<HttpResponse(const HttpRequest&)> RouteHandler;

/**
 * @class RestApiServer
 * @brief HTTP REST API server for dashboard
 * 
 * Provides RESTful access to dashboard data including:
 * - Topics and nodes information
 * - Metrics and statistics
 * - Configuration and settings
 * - Recording management
 */
class RestApiServer {
public:
    RestApiServer(int port = 8080);
    ~RestApiServer();

    // Delete copy operations
    RestApiServer(const RestApiServer&) = delete;
    RestApiServer& operator=(const RestApiServer&) = delete;

    /**
     * @brief Start the REST API server
     * @return true if server started successfully
     */
    bool start();

    /**
     * @brief Stop the REST API server
     */
    void stop();

    /**
     * @brief Check if server is running
     */
    bool is_running() const { return running_; }

    /**
     * @brief Get server port
     */
    int get_port() const { return port_; }

    /**
     * @brief Register a route handler
     * @param method HTTP method (GET, POST, etc.)
     * @param path Route path (e.g., "/api/topics")
     * @param handler Function to handle requests
     */
    void register_route(const std::string& method, 
                       const std::string& path, 
                       RouteHandler handler);

    /**
     * @brief Register GET endpoint
     */
    void get(const std::string& path, RouteHandler handler) {
        register_route("GET", path, handler);
    }

    /**
     * @brief Register POST endpoint
     */
    void post(const std::string& path, RouteHandler handler) {
        register_route("POST", path, handler);
    }

    /**
     * @brief Register PUT endpoint
     */
    void put(const std::string& path, RouteHandler handler) {
        register_route("PUT", path, handler);
    }

    /**
     * @brief Register DELETE endpoint
     */
    void del(const std::string& path, RouteHandler handler) {
        register_route("DELETE", path, handler);
    }

    /**
     * @brief Set CORS headers for cross-origin requests
     */
    void enable_cors();

    /**
     * @brief Set authentication token requirement
     */
    void require_auth(const std::string& token);

    /**
     * @brief Get last error message
     */
    std::string get_last_error() const { return last_error_; }

private:
    /**
     * @brief Handle incoming HTTP request
     */
    HttpResponse handle_request_(const HttpRequest& request);

    /**
     * @brief Parse request path and extract route parameters
     */
    bool match_route_(const std::string& method, 
                     const std::string& path,
                     RouteHandler& handler);

    int port_;
    bool running_ = false;
    std::string last_error_;
    std::string auth_token_;
    bool enable_cors_ = true;
    
    // Route storage: method -> path -> handler
    std::map<std::string, std::map<std::string, RouteHandler>> routes_;
    
    // Server implementation (stored as void* to hide implementation)
    void* server_impl_;
};

}  // namespace ros2_dashboard::server
