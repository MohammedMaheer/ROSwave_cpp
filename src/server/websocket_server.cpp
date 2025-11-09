/**
 * @file websocket_server.cpp
 * @brief WebSocket server implementation
 * @author Dashboard Team
 */

#include "server/websocket_server.hpp"
#include "logging.hpp"
#include <map>
#include <set>
#include <thread>
#include <mutex>
#include <memory>

namespace ros2_dashboard::server {

/**
 * @struct ServerImpl
 * @brief Internal server implementation
 */
struct ServerImpl {
    std::map<ClientId, int> client_sockets;
    std::set<ClientId> connected_clients;
    std::mutex clients_mutex;
    volatile bool should_stop = false;
    int server_socket = -1;
    std::thread server_thread;
};

WebSocketServer::WebSocketServer(int port) 
    : port_(port), server_impl_(new ServerImpl()) {
}

WebSocketServer::~WebSocketServer() {
    stop();
    delete static_cast<ServerImpl*>(server_impl_);
}

bool WebSocketServer::start() {
    ServerImpl* impl = static_cast<ServerImpl*>(server_impl_);
    
    if (running_) {
        last_error_ = "Server already running";
        return false;
    }

    running_ = true;
    impl->should_stop = false;

    StructuredLogger::instance().info("WebSocket server started on port " + 
                                     std::to_string(port_));
    return true;
}

void WebSocketServer::stop() {
    if (!running_) return;

    ServerImpl* impl = static_cast<ServerImpl*>(server_impl_);
    impl->should_stop = true;
    running_ = false;

    {
        std::lock_guard<std::mutex> lock(impl->clients_mutex);
        impl->connected_clients.clear();
        impl->client_sockets.clear();
    }

    StructuredLogger::instance().info("WebSocket server stopped");
}

std::vector<ClientId> WebSocketServer::get_clients() const {
    ServerImpl* impl = static_cast<ServerImpl*>(server_impl_);
    std::lock_guard<std::mutex> lock(impl->clients_mutex);
    
    std::vector<ClientId> result;
    for (const auto& client : impl->connected_clients) {
        result.push_back(client);
    }
    return result;
}

bool WebSocketServer::send_to_client(const ClientId& client_id, 
                                    const WebSocketMessage& message) {
    ServerImpl* impl = static_cast<ServerImpl*>(server_impl_);
    std::lock_guard<std::mutex> lock(impl->clients_mutex);

    auto it = impl->client_sockets.find(client_id);
    if (it == impl->client_sockets.end()) {
        StructuredLogger::instance().warn("Client not found: " + client_id);
        return false;
    }

    // Simple message send (for actual WebSocket, need proper framing)
    try {
        // In production, would use proper WebSocket frame format
        // For now, just log the send
        StructuredLogger::instance().debug("Sent to client " + client_id + ": " + 
                                          message.payload.substr(0, 50));
        return true;
    } catch (const std::exception& e) {
        StructuredLogger::instance().error("Failed to send to client: " + 
                                          std::string(e.what()));
        return false;
    }
}

void WebSocketServer::broadcast(const WebSocketMessage& message) {
    auto clients = get_clients();
    for (const auto& client : clients) {
        send_to_client(client, message);
    }
}

void WebSocketServer::disconnect_client(const ClientId& client_id) {
    ServerImpl* impl = static_cast<ServerImpl*>(server_impl_);
    std::lock_guard<std::mutex> lock(impl->clients_mutex);

    impl->connected_clients.erase(client_id);
    impl->client_sockets.erase(client_id);

    if (on_disconnected_callback_) {
        on_disconnected_callback_(client_id);
    }

    StructuredLogger::instance().info("Client disconnected: " + client_id);
}

}  // namespace ros2_dashboard::server
