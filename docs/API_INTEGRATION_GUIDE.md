# API Integration Guide

## Overview

This guide explains how to connect the REST API and WebSocket server to the existing dashboard UI components (MainWindow, tabs, managers).

## Architecture

```
MainWindow (Qt UI)
    ├── MetricsTab
    │   ├── MetricsCollector → Metrics data
    │   └── AlertManager → Alerts/Health
    ├── TopicsTab
    │   ├── TopicMonitor → Topic information
    │   └── DependencyGraph → Relationships
    ├── NodesTab
    │   └── ROS2Manager → Node information
    └── [other tabs]

REST API Server (Port 8080)
    ├── /api/metrics → GET from MetricsCollector
    ├── /api/topics → GET from TopicMonitor
    ├── /api/nodes → GET from ROS2Manager
    └── /api/health → GET from AlertManager

WebSocket Server (Port 8081)
    ├── metrics_update (every 100ms)
    ├── topic_discovered
    └── alert_triggered
```

## Step 1: Access MainWindow from API Routes

### Problem

API route handlers currently have no access to MainWindow or its data members:

```cpp
// Current: stub implementation
HttpResponse get_metrics(const HttpRequest& request) {
    // TODO: Get data from MainWindow
    return HttpResponse::ok(nlohmann::json{{"cpu_percent", 0}});
}
```

### Solution

Pass MainWindow pointer to ApiRoutes:

#### Modify include/server/api_routes.hpp

```cpp
#pragma once

#include <string>
#include <functional>
#include "rest_api_server.hpp"

class MainWindow;  // Forward declaration

class ApiRoutes {
public:
    // Pass MainWindow instance to route handlers
    static void setup_routes(RestApiServer& server, MainWindow* main_window);
};
```

#### Modify src/server/api_routes.cpp

```cpp
#include "api_routes.hpp"
#include "rest_api_server.hpp"
#include <nlohmann/json.hpp>

// Include MainWindow and related headers
// #include "gui/main_window.hpp"
// #include "metrics_collector.hpp"
// #include "topic_monitor.hpp"
// #include "ros2_manager.hpp"
// #include "alert_manager.hpp"

using json = nlohmann::json;

void ApiRoutes::setup_routes(RestApiServer& server, MainWindow* main_window) {
    // All handlers now have access to main_window
    
    // GET /api/metrics
    server.get("/api/metrics", [main_window](const HttpRequest& req) {
        if (!main_window) {
            return HttpResponse::internal_error("MainWindow not available");
        }
        
        // Get metrics from MetricsCollector
        // auto metrics = main_window->metrics_tab_->metrics_collector_->get_current_metrics();
        
        json response = {
            {"cpu_percent", 0},
            {"memory_mb", 0},
            {"temperature_c", 0},
            {"timestamp", 0}
        };
        return HttpResponse::ok(response);
    });
    
    // ... other routes
}
```

### Update main.cpp

In `main.cpp` where the API server is started:

```cpp
#include "server/rest_api_server.hpp"
#include "server/websocket_server.hpp"
#include "server/api_routes.hpp"

// In MainWindow::MainWindow() or initialization
RestApiServer api_server(8080);
WebSocketServer ws_server(8081);

// Pass this pointer to setup routes
ApiRoutes::setup_routes(api_server, this);

api_server.start();
ws_server.start();
```

## Step 2: Implement Data Extraction Methods

### Get Metrics from MetricsCollector

```cpp
// In src/server/api_routes.cpp

server.get("/api/metrics", [main_window](const HttpRequest& req) {
    auto current_metrics = main_window->metrics_tab_->metrics_collector_
        ->get_current_metrics();
    
    json response = {
        {"cpu_percent", current_metrics.cpu_percent},
        {"memory_mb", current_metrics.memory_mb},
        {"temperature_c", current_metrics.temperature_c},
        {"disk_usage_percent", current_metrics.disk_usage_percent},
        {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}
    };
    
    return HttpResponse::ok(response);
});
```

### Get Topics from TopicMonitor

```cpp
server.get("/api/topics", [main_window](const HttpRequest& req) {
    auto topics = main_window->topics_tab_->topic_monitor_->get_topics();
    
    json topics_array = json::array();
    for (const auto& topic : topics) {
        topics_array.push_back({
            {"name", topic.name},
            {"type", topic.message_type},
            {"publishers", topic.publisher_count},
            {"subscribers", topic.subscriber_count},
            {"frequency_hz", topic.frequency_hz},
            {"message_size_bytes", topic.message_size}
        });
    }
    
    return HttpResponse::ok({
        {"topics", topics_array},
        {"count", topics_array.size()}
    });
});
```

### Get Nodes from ROS2Manager

```cpp
server.get("/api/nodes", [main_window](const HttpRequest& req) {
    auto nodes = main_window->nodes_tab_->ros2_manager_->get_nodes();
    
    json nodes_array = json::array();
    for (const auto& node : nodes) {
        nodes_array.push_back({
            {"name", node.name},
            {"namespace", node.namespace_},
            {"publishers", node.publisher_count},
            {"subscribers", node.subscriber_count},
            {"services", node.service_count}
        });
    }
    
    return HttpResponse::ok({
        {"nodes", nodes_array},
        {"count", nodes_array.size()}
    });
});
```

### Get Health Status from AlertManager

```cpp
server.get("/api/health", [main_window](const HttpRequest& req) {
    auto alerts = main_window->metrics_tab_->alert_manager_->get_all_alerts();
    
    json checks = json::object();
    checks["cpu"] = {
        {"status", "ok"},
        {"message", "CPU usage within limits"}
    };
    checks["memory"] = {
        {"status", "ok"},
        {"message", "Memory usage normal"}
    };
    
    // Check for any critical alerts
    bool has_critical = false;
    for (const auto& alert : alerts) {
        if (alert.severity == AlertSeverity::CRITICAL) {
            has_critical = true;
            checks[alert.name] = {
                {"status", "critical"},
                {"message", alert.message}
            };
        }
    }
    
    return HttpResponse::ok({
        {"status", has_critical ? "warning" : "ok"},
        {"checks", checks}
    });
});
```

## Step 3: WebSocket Real-Time Updates

### Broadcast Metrics Periodically

In MainWindow or MetricsCollector:

```cpp
// Timer to broadcast metrics every 100ms
QTimer* metrics_timer = new QTimer(this);
connect(metrics_timer, &QTimer::timeout, [this]() {
    auto metrics = metrics_collector_->get_current_metrics();
    
    json update = {
        {"type", "metrics_update"},
        {"cpu_percent", metrics.cpu_percent},
        {"memory_mb", metrics.memory_mb},
        {"temperature_c", metrics.temperature_c},
        {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}
    };
    
    ws_server_->broadcast_json("metrics", update);
});
metrics_timer->start(100);  // Every 100ms
```

### Broadcast Topic Discovery

When a new topic is discovered:

```cpp
void TopicMonitor::on_topic_discovered(const TopicInfo& topic) {
    json announcement = {
        {"type", "topic_discovered"},
        {"topic", {
            {"name", topic.name},
            {"type", topic.message_type}
        }}
    };
    
    ws_server_->broadcast_json("topics", announcement);
}
```

### Broadcast Alerts

When alert is triggered:

```cpp
void AlertManager::on_alert_triggered(const Alert& alert) {
    json alert_msg = {
        {"type", "alert_triggered"},
        {"name", alert.name},
        {"severity", alert_severity_to_string(alert.severity)},
        {"message", alert.message},
        {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}
    };
    
    ws_server_->broadcast_json("alerts", alert_msg);
}
```

## Step 4: Handle Query Parameters

### Filter Topics

```cpp
server.get("/api/topics", [main_window](const HttpRequest& req) {
    auto topics = main_window->topics_tab_->topic_monitor_->get_topics();
    
    // Check for filter parameter
    std::string filter;
    if (req.query_params.count("filter")) {
        filter = req.query_params.at("filter");
    }
    
    json topics_array = json::array();
    for (const auto& topic : topics) {
        // Filter by name if provided
        if (!filter.empty() && 
            topic.name.find(filter) == std::string::npos) {
            continue;
        }
        
        topics_array.push_back({
            {"name", topic.name},
            {"type", topic.message_type},
            {"publishers", topic.publisher_count},
            {"subscribers", topic.subscriber_count}
        });
    }
    
    return HttpResponse::ok({
        {"topics", topics_array},
        {"filter", filter},
        {"count", topics_array.size()}
    });
});
```

### Metrics History

```cpp
server.get("/api/metrics/history", [main_window](const HttpRequest& req) {
    // Get duration from query parameter
    int duration_seconds = 300;  // default 5 minutes
    if (req.query_params.count("duration")) {
        try {
            duration_seconds = std::stoi(req.query_params.at("duration"));
        } catch (...) {}
    }
    
    // Get historical data
    auto history = main_window->metrics_tab_->metrics_history_
        ->get_history(duration_seconds);
    
    json data = json::array();
    for (const auto& entry : history) {
        data.push_back({
            {"timestamp", entry.timestamp},
            {"cpu_percent", entry.cpu_percent},
            {"memory_mb", entry.memory_mb},
            {"temperature_c", entry.temperature_c}
        });
    }
    
    return HttpResponse::ok({
        {"data", data},
        {"duration_seconds", duration_seconds},
        {"points", data.size()}
    });
});
```

## Step 5: Error Handling

### Connection Error Handling

```cpp
server.get("/api/topics", [main_window](const HttpRequest& req) {
    if (!main_window) {
        return HttpResponse::internal_error({
            {"error", "MainWindow not available"}
        });
    }
    
    try {
        auto topics = main_window->topics_tab_->topic_monitor_->get_topics();
        // ... build response
    } catch (const std::exception& e) {
        return HttpResponse::internal_error({
            {"error", e.what()}
        });
    }
});
```

### ROS2 Connection Check

```cpp
server.get("/api/status", [main_window](const HttpRequest& req) {
    bool ros2_available = false;
    std::string ros_version;
    
    try {
        auto context = main_window->ros2_manager_->get_context();
        if (context) {
            ros2_available = true;
            // Get ROS version
        }
    } catch (...) {
        ros2_available = false;
    }
    
    return HttpResponse::ok({
        {"status", ros2_available ? "connected" : "disconnected"},
        {"ros2_detected", ros2_available},
        {"ros_version", ros_version},
        {"uptime", get_uptime_seconds()}
    });
});
```

## Step 6: Testing

### Test Individual Endpoints

```bash
# Test metrics
curl http://localhost:8080/api/metrics | jq

# Test topics with filter
curl 'http://localhost:8080/api/topics?filter=sensor' | jq

# Test nodes
curl http://localhost:8080/api/nodes | jq

# Test health
curl http://localhost:8080/api/health | jq

# Test service list
curl http://localhost:8080/api/services | jq
```

### Test WebSocket Updates

```bash
# Install wscat
npm install -g wscat

# Connect and listen
wscat -c ws://localhost:8081

# Should receive metrics updates automatically
```

### Python Test Script

```python
#!/usr/bin/env python3
import requests
import websocket
import json
import time

BASE_URL = "http://localhost:8080/api"
WS_URL = "ws://localhost:8081"

def test_rest_api():
    """Test REST endpoints"""
    print("Testing REST API...")
    
    # Test metrics
    resp = requests.get(f"{BASE_URL}/metrics")
    print(f"Metrics: {resp.json()}")
    
    # Test topics
    resp = requests.get(f"{BASE_URL}/topics")
    print(f"Topics: {resp.json()}")
    
    # Test nodes
    resp = requests.get(f"{BASE_URL}/nodes")
    print(f"Nodes: {resp.json()}")
    
    # Test health
    resp = requests.get(f"{BASE_URL}/health")
    print(f"Health: {resp.json()}")

def test_websocket():
    """Test WebSocket real-time updates"""
    print("Testing WebSocket...")
    
    def on_message(ws, message):
        data = json.loads(message)
        print(f"Update: {data}")
    
    def on_error(ws, error):
        print(f"Error: {error}")
    
    def on_close(ws, close_status_code, close_msg):
        print("Connection closed")
    
    def on_open(ws):
        print("Connected to WebSocket")
        # Subscribe to metrics
        ws.send(json.dumps({
            "type": "subscribe",
            "channel": "metrics"
        }))
    
    ws = websocket.WebSocketApp(
        WS_URL,
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )
    
    # Run for 10 seconds
    import threading
    thread = threading.Thread(target=ws.run_forever)
    thread.daemon = True
    thread.start()
    
    time.sleep(10)
    ws.close()

if __name__ == "__main__":
    test_rest_api()
    print()
    test_websocket()
```

## Step 7: Performance Considerations

### Caching Frequently Accessed Data

```cpp
// Cache topic list for 1 second
auto cached_topics = std::make_pair(
    std::chrono::steady_clock::now(),
    main_window->topics_tab_->topic_monitor_->get_topics()
);

server.get("/api/topics", [cached_topics](const HttpRequest& req) {
    auto now = std::chrono::steady_clock::now();
    auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - cached_topics.first
    ).count();
    
    if (age < 1000) {  // Use cache if less than 1 second old
        // Use cached data
    } else {
        // Fetch fresh data
    }
});
```

### Limiting Response Size

```cpp
server.get("/api/metrics/history", [main_window](const HttpRequest& req) {
    int duration = 300;
    int max_points = 1000;
    
    auto history = main_window->metrics_tab_->metrics_history_
        ->get_history(duration, max_points);
    
    // Downsample if needed
    if (history.size() > max_points) {
        // Aggregate or skip points
    }
    
    return HttpResponse::ok({{"data", history}});
});
```

### Rate Limiting

```cpp
#include <map>
#include <chrono>

std::map<std::string, std::chrono::steady_clock::time_point> last_request;
const int REQUEST_LIMIT_PER_SECOND = 100;

bool check_rate_limit(const std::string& client_ip) {
    auto now = std::chrono::steady_clock::now();
    
    if (last_request.count(client_ip)) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_request[client_ip]
        ).count();
        
        if (elapsed < 1000 / REQUEST_LIMIT_PER_SECOND) {
            return false;  // Rate limit exceeded
        }
    }
    
    last_request[client_ip] = now;
    return true;
}
```

## Debugging

### Enable API Logging

Add to RestApiServer:

```cpp
class RestApiServer {
private:
    bool log_enabled_ = true;
    
    void log_request(const HttpRequest& req) {
        if (log_enabled_) {
            std::cout << "[API] " << req.method << " " << req.path << std::endl;
        }
    }
};
```

### WebSocket Debug Output

```cpp
server.on_message([](const WebSocketMessage& msg, const std::string& client_id) {
    std::cout << "[WS] Client " << client_id << ": " 
              << msg.data << std::endl;
});
```

### Performance Profiling

```bash
# Profile with perf
perf record -F 99 -p $(pgrep ros2_dashboard) -g -- sleep 30
perf report

# Monitor with top
top -p $(pgrep ros2_dashboard)

# Monitor API response time
curl -w 'Response time: %{time_total}s\n' http://localhost:8080/api/metrics
```

## See Also

- [REST API Documentation](REST_WEBSOCKET_API.md)
- [Web UI Deployment](WEB_DEPLOYMENT.md)
- [Developer Guide](DEVELOPER.md)
