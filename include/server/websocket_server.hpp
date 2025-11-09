/**
 * @file websocket_server.hpp
 * @brief WebSocket server for real-time dashboard updates
 * @author Dashboard Team
 */

#pragma once

#include <string>
#include <memory>
#include <vector>
#include <functional>
#include <nlohmann/json.hpp>

namespace ros2_dashboard::server {

using json = nlohmann::json;

/**
 * @struct WebSocketMessage
 * @brief WebSocket message frame
 */
struct WebSocketMessage {
    enum Type {
        TEXT,      // Text message
        BINARY,    // Binary data
        PING,      // Ping frame
        PONG,      // Pong frame
        CLOSE      // Connection close
    };

    Type type = TEXT;
    std::string payload;
    
    /**
     * @brief Parse JSON message
     */
    json as_json() const {
        return json::parse(payload);
    }
    
    /**
     * @brief Create JSON text message
     */
    static WebSocketMessage json_message(const json& data) {
        WebSocketMessage msg;
        msg.type = TEXT;
        msg.payload = data.dump();
        return msg;
    }
};

/**
 * @typedef ClientId
 * @brief Unique client identifier
 */
typedef std::string ClientId;

/**
 * @typedef OnMessageCallback
 * @brief Callback when message received
 */
typedef std::function<void(const ClientId&, const WebSocketMessage&)> OnMessageCallback;

/**
 * @typedef OnClientConnectedCallback
 * @brief Callback when client connects
 */
typedef std::function<void(const ClientId&)> OnClientConnectedCallback;

/**
 * @typedef OnClientDisconnectedCallback
 * @brief Callback when client disconnects
 */
typedef std::function<void(const ClientId&)> OnClientDisconnectedCallback;

/**
 * @class WebSocketServer
 * @brief WebSocket server for real-time updates
 * 
 * Enables real-time bidirectional communication with connected clients:
 * - Push metrics updates
 * - Stream topic messages
 * - Receive commands from clients
 * - Broadcast events
 */
class WebSocketServer {
public:
    WebSocketServer(int port = 8081);
    ~WebSocketServer();

    // Delete copy operations
    WebSocketServer(const WebSocketServer&) = delete;
    WebSocketServer& operator=(const WebSocketServer&) = delete;

    /**
     * @brief Start the WebSocket server
     * @return true if server started successfully
     */
    bool start();

    /**
     * @brief Stop the WebSocket server
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
     * @brief Get list of connected clients
     */
    std::vector<ClientId> get_clients() const;

    /**
     * @brief Get number of connected clients
     */
    size_t client_count() const { return get_clients().size(); }

    /**
     * @brief Send message to specific client
     * @param client_id Target client
     * @param message Message to send
     * @return true if sent successfully
     */
    bool send_to_client(const ClientId& client_id, const WebSocketMessage& message);

    /**
     * @brief Broadcast message to all connected clients
     * @param message Message to broadcast
     */
    void broadcast(const WebSocketMessage& message);

    /**
     * @brief Broadcast JSON message to all clients
     */
    void broadcast_json(const json& data) {
        broadcast(WebSocketMessage::json_message(data));
    }

    /**
     * @brief Disconnect specific client
     */
    void disconnect_client(const ClientId& client_id);

    // Callbacks

    /**
     * @brief Register callback for incoming messages
     */
    void on_message(OnMessageCallback callback) {
        on_message_callback_ = callback;
    }

    /**
     * @brief Register callback for client connect
     */
    void on_client_connected(OnClientConnectedCallback callback) {
        on_connected_callback_ = callback;
    }

    /**
     * @brief Register callback for client disconnect
     */
    void on_client_disconnected(OnClientDisconnectedCallback callback) {
        on_disconnected_callback_ = callback;
    }

    /**
     * @brief Get last error message
     */
    std::string get_last_error() const { return last_error_; }

private:
    /**
     * @brief Handle client connection
     */
    void handle_client_connected_(const ClientId& client_id);

    /**
     * @brief Handle client message
     */
    void handle_message_(const ClientId& client_id, const WebSocketMessage& msg);

    /**
     * @brief Handle client disconnect
     */
    void handle_client_disconnected_(const ClientId& client_id);

    int port_;
    bool running_ = false;
    std::string last_error_;

    OnMessageCallback on_message_callback_;
    OnClientConnectedCallback on_connected_callback_;
    OnClientDisconnectedCallback on_disconnected_callback_;

    // Server implementation (stored as void* to hide implementation)
    void* server_impl_;
};

}  // namespace ros2_dashboard::server
