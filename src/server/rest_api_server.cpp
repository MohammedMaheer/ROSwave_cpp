/**
 * @file rest_api_server.cpp
 * @brief REST API server implementation
 * @author Dashboard Team
 */

#include "server/rest_api_server.hpp"
#include "logging.hpp"
#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <map>
#include <cstring>

#ifdef _WIN32
    #include <winsock2.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <arpa/inet.h>
    #include <netinet/in.h>
    #include <sys/socket.h>
    #include <unistd.h>
    #include <cstring>
#endif

namespace ros2_dashboard::server {

/**
 * @struct ServerImpl
 * @brief Internal server implementation
 */
struct ServerImpl {
    int server_socket = -1;
    std::thread server_thread;
    volatile bool should_stop = false;
    std::mutex routes_mutex;
    std::map<std::string, std::map<std::string, RouteHandler>> routes;
    bool cors_enabled = true;
};

RestApiServer::RestApiServer(int port) 
    : port_(port), server_impl_(new ServerImpl()) {
}

RestApiServer::~RestApiServer() {
    stop();
    delete static_cast<ServerImpl*>(server_impl_);
}

void RestApiServer::register_route(const std::string& method, 
                                  const std::string& path, 
                                  RouteHandler handler) {
    ServerImpl* impl = static_cast<ServerImpl*>(server_impl_);
    std::lock_guard<std::mutex> lock(impl->routes_mutex);
    impl->routes[method][path] = handler;
    StructuredLogger::instance().info("Registered route: " + method + " " + path);
}

void RestApiServer::enable_cors() {
    ServerImpl* impl = static_cast<ServerImpl*>(server_impl_);
    impl->cors_enabled = true;
}

void RestApiServer::require_auth(const std::string& token) {
    auth_token_ = token;
}

bool RestApiServer::start() {
    ServerImpl* impl = static_cast<ServerImpl*>(server_impl_);
    
    if (running_) {
        last_error_ = "Server already running";
        return false;
    }

    // Create socket
#ifdef _WIN32
    WSADATA wsa_data;
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
        last_error_ = "WSAStartup failed";
        return false;
    }
#endif

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        last_error_ = "Failed to create socket";
        return false;
    }

    // Set socket options
    int reuse = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, 
                   (const char*)&reuse, sizeof(reuse)) < 0) {
        last_error_ = "Failed to set socket options";
#ifdef _WIN32
        closesocket(sock);
#else
        close(sock);
#endif
        return false;
    }

    // Bind socket
    struct sockaddr_in server_addr;
    std::memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(port_);

    if (bind(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        last_error_ = "Failed to bind socket to port " + std::to_string(port_);
#ifdef _WIN32
        closesocket(sock);
#else
        close(sock);
#endif
        return false;
    }

    // Listen
    if (listen(sock, SOMAXCONN) < 0) {
        last_error_ = "Failed to listen on socket";
#ifdef _WIN32
        closesocket(sock);
#else
        close(sock);
#endif
        return false;
    }

    impl->server_socket = sock;
    impl->should_stop = false;
    running_ = true;

    // Start server thread
    impl->server_thread = std::thread([this, impl]() {
        while (!impl->should_stop) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            int client_sock = accept(impl->server_socket, 
                                    (struct sockaddr*)&client_addr, 
                                    &client_len);
            
            if (client_sock < 0 && !impl->should_stop) {
                continue;
            }
            if (client_sock < 0) break;

            // Handle client in separate thread
            std::thread([this, client_sock]() {
                char buffer[4096] = {0};
                int bytes_read = recv(client_sock, buffer, sizeof(buffer) - 1, 0);
                
                if (bytes_read > 0) {
                    // Parse HTTP request
                    std::string request_str(buffer, bytes_read);
                    
                    // Simple HTTP parsing
                    std::istringstream iss(request_str);
                    std::string method, path, version;
                    iss >> method >> path >> version;

                    HttpRequest request;
                    request.method = method;
                    request.path = path;

                    // Extract query parameters
                    size_t query_pos = path.find('?');
                    if (query_pos != std::string::npos) {
                        std::string query = path.substr(query_pos + 1);
                        request.path = path.substr(0, query_pos);
                        
                        // Parse query string
                        std::istringstream query_ss(query);
                        std::string pair;
                        while (std::getline(query_ss, pair, '&')) {
                            size_t eq = pair.find('=');
                            if (eq != std::string::npos) {
                                request.query_params[pair.substr(0, eq)] = 
                                    pair.substr(eq + 1);
                            }
                        }
                    }

                    // Parse headers
                    std::string line;
                    while (std::getline(iss, line)) {
                        size_t colon = line.find(':');
                        if (colon != std::string::npos) {
                            std::string key = line.substr(0, colon);
                            std::string value = line.substr(colon + 2);
                            // Trim
                            if (!value.empty() && value.back() == '\r') {
                                value.pop_back();
                            }
                            request.headers[key] = value;
                        }
                    }

                    // Get body for POST/PUT
                    if (method == "POST" || method == "PUT") {
                        // Find empty line separator
                        size_t header_end = request_str.find("\r\n\r\n");
                        if (header_end != std::string::npos) {
                            request.body = request_str.substr(header_end + 4);
                        }
                    }

                    // Handle request
                    HttpResponse response = handle_request_(request);

                    // Build HTTP response
                    std::ostringstream http_response;
                    http_response << "HTTP/1.1 " << response.status_code << " OK\r\n";
                    
                    if (enable_cors_) {
                        http_response << "Access-Control-Allow-Origin: *\r\n";
                        http_response << "Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS\r\n";
                        http_response << "Access-Control-Allow-Headers: Content-Type, Authorization\r\n";
                    }
                    
                    for (const auto& header : response.headers) {
                        http_response << header.first << ": " << header.second << "\r\n";
                    }
                    
                    http_response << "Content-Length: " << response.body.length() << "\r\n";
                    http_response << "\r\n";
                    http_response << response.body;

                    std::string response_str = http_response.str();
                    send(client_sock, response_str.c_str(), response_str.length(), 0);
                }

#ifdef _WIN32
                closesocket(client_sock);
#else
                close(client_sock);
#endif
            }).detach();
        }
    });

    StructuredLogger::instance().info("REST API server started on port " + 
                                     std::to_string(port_));
    return true;
}

void RestApiServer::stop() {
    if (!running_) return;

    ServerImpl* impl = static_cast<ServerImpl*>(server_impl_);
    impl->should_stop = true;
    running_ = false;

    if (impl->server_socket >= 0) {
#ifdef _WIN32
        closesocket(impl->server_socket);
        WSACleanup();
#else
        close(impl->server_socket);
#endif
        impl->server_socket = -1;
    }

    if (impl->server_thread.joinable()) {
        impl->server_thread.join();
    }

    StructuredLogger::instance().info("REST API server stopped");
}

HttpResponse RestApiServer::handle_request_(const HttpRequest& request) {
    // Check authentication
    if (!auth_token_.empty()) {
        auto auth_it = request.headers.find("Authorization");
        if (auth_it == request.headers.end() || 
            auth_it->second.find(auth_token_) == std::string::npos) {
            return HttpResponse::internal_error("Unauthorized");
        }
    }

    // Handle OPTIONS for CORS preflight
    if (request.method == "OPTIONS") {
        HttpResponse response;
        response.status_code = 200;
        return response;
    }

    // Find matching route
    ServerImpl* impl = static_cast<ServerImpl*>(server_impl_);
    std::lock_guard<std::mutex> lock(impl->routes_mutex);

    auto method_it = impl->routes.find(request.method);
    if (method_it != impl->routes.end()) {
        auto path_it = method_it->second.find(request.path);
        if (path_it != method_it->second.end()) {
            try {
                return path_it->second(request);
            } catch (const std::exception& e) {
                StructuredLogger::instance().error("Route handler exception: " + 
                                                  std::string(e.what()));
                return HttpResponse::internal_error(e.what());
            }
        }
    }

    return HttpResponse::not_found();
}

}  // namespace ros2_dashboard::server
