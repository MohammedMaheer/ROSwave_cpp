# REST API & WebSocket Documentation

## Overview

The ROS2 Dashboard provides a comprehensive REST API and WebSocket server for remote access and real-time updates. This enables:

- **Remote Dashboard Access** - Access the dashboard from a web browser
- **Data Integration** - External tools can fetch metrics and topic information
- **Real-time Streaming** - WebSocket support for live data updates
- **Programmatic Control** - Automate dashboard operations via API

## Starting the Servers

The REST API and WebSocket servers start automatically with the dashboard:

```bash
ros2_dashboard --api-port 8080 --ws-port 8081
```

## REST API Reference

### Base URL
```
http://localhost:8080/api
```

### Common Response Format

All endpoints return JSON responses with consistent structure:

```json
{
    "data": {},           // Response data
    "status": "success",  // "success" or "error"
    "timestamp": 1234567890,
    "error": null
}
```

### Error Responses

```json
{
    "error": "Description of what went wrong",
    "status": "error",
    "timestamp": 1234567890
}
```

### Authentication

Include authentication token in `Authorization` header:

```bash
curl -H "Authorization: Bearer YOUR_TOKEN" http://localhost:8080/api/topics
```

---

## Topics Endpoints

### List All Topics

**Request:**
```http
GET /api/topics
```

**Query Parameters:**
- `filter` - Filter topics by name (regex)
- `type` - Filter by message type
- `sort` - Sort by: name, rate, size, type

**Response:**
```json
{
    "topics": [
        {
            "name": "/sensor/imu",
            "type": "sensor_msgs/msg/Imu",
            "publishers": 1,
            "subscribers": 2,
            "frequency_hz": 100.0,
            "message_size_bytes": 456,
            "last_message_time": 1234567890123
        }
    ],
    "count": 1,
    "timestamp": 1234567890
}
```

### Get Topic Details

**Request:**
```http
GET /api/topics/{topic_name}
```

**Path Parameters:**
- `topic_name` - Topic name (URL encoded)

**Response:**
```json
{
    "name": "/sensor/imu",
    "type": "sensor_msgs/msg/Imu",
    "publishers": [
        {
            "node_name": "/sensor_driver",
            "node_namespace": "/"
        }
    ],
    "subscribers": [
        {
            "node_name": "/planner",
            "node_namespace": "/"
        }
    ],
    "frequency_hz": 100.0,
    "message_size_bytes": 456,
    "total_messages_published": 150000,
    "last_message_time": 1234567890123,
    "message_definition": "..."
}
```

### Subscribe to Topic

**Request:**
```http
POST /api/topics/{topic_name}/subscribe
```

**Request Body:**
```json
{
    "queue_size": 10,
    "callback_url": "http://your-server/callback"
}
```

**Response:**
```json
{
    "subscription_id": "sub_12345",
    "topic_name": "/sensor/imu",
    "status": "subscribed"
}
```

### Unsubscribe from Topic

**Request:**
```http
DELETE /api/subscriptions/{subscription_id}
```

---

## Nodes Endpoints

### List All Nodes

**Request:**
```http
GET /api/nodes
```

**Response:**
```json
{
    "nodes": [
        {
            "name": "sensor_driver",
            "namespace": "/",
            "full_name": "/sensor_driver",
            "publishers": ["/sensor/imu", "/sensor/camera"],
            "subscribers": [],
            "services": [],
            "parameters": ["rate", "debug"]
        }
    ],
    "count": 1
}
```

### Get Node Details

**Request:**
```http
GET /api/nodes/{node_name}
```

**Response:**
```json
{
    "name": "sensor_driver",
    "namespace": "/",
    "publishers": [
        {
            "topic": "/sensor/imu",
            "type": "sensor_msgs/msg/Imu",
            "frequency_hz": 100.0
        }
    ],
    "subscribers": [],
    "services": [
        {
            "name": "/sensor_driver/set_mode",
            "type": "std_srvs/srv/SetBool"
        }
    ],
    "parameters": {
        "rate": {
            "type": "int",
            "value": 100
        },
        "debug": {
            "type": "bool",
            "value": false
        }
    }
}
```

---

## Metrics Endpoints

### Get Current Metrics

**Request:**
```http
GET /api/metrics
```

**Response:**
```json
{
    "cpu_percent": 45.2,
    "memory_mb": 1024.5,
    "temperature_c": 65.5,
    "disk_usage_percent": 75.0,
    "timestamp": 1234567890123
}
```

### Get Metrics History

**Request:**
```http
GET /api/metrics/history?duration_minutes=60
```

**Query Parameters:**
- `duration_minutes` - Historical period (default: 60)
- `resolution` - Data points per hour (default: 60)

**Response:**
```json
{
    "data_points": [
        {
            "timestamp": 1234567890000,
            "cpu_percent": 45.2,
            "memory_mb": 1024.5,
            "temperature_c": 65.5
        }
    ],
    "count": 60,
    "start_time": 1234564290000,
    "end_time": 1234567890000
}
```

---

## Services Endpoints

### List All Services

**Request:**
```http
GET /api/services
```

**Response:**
```json
{
    "services": [
        {
            "name": "/sensor_driver/set_mode",
            "type": "std_srvs/srv/SetBool",
            "server_node": "/sensor_driver"
        }
    ],
    "count": 1
}
```

### Call Service

**Request:**
```http
POST /api/services/{service_name}/call
```

**Request Body:**
```json
{
    "request": {
        "data": true
    }
}
```

**Response:**
```json
{
    "service_name": "/sensor_driver/set_mode",
    "success": true,
    "message": "Service call successful",
    "response": {
        "success": true,
        "message": "Mode set"
    }
}
```

---

## System Endpoints

### Get Dashboard Status

**Request:**
```http
GET /api/status
```

**Response:**
```json
{
    "status": "running",
    "version": "2.0.0",
    "uptime_seconds": 3600,
    "ros2_domain_id": 0,
    "ros2_detected": true,
    "ros_version": "Humble",
    "dashboard_start_time": 1234563290
}
```

### Get Health Status

**Request:**
```http
GET /api/health
```

**Response:**
```json
{
    "status": "healthy",
    "checks": {
        "ros2_connectivity": {
            "status": "ok",
            "message": "Connected to ROS2 network"
        },
        "memory": {
            "status": "ok",
            "message": "Memory usage normal"
        },
        "cpu": {
            "status": "warning",
            "message": "CPU usage above 50%"
        },
        "disk": {
            "status": "ok",
            "message": "Disk space available"
        }
    },
    "timestamp": 1234567890
}
```

---

## WebSocket API

### Connection

```javascript
const ws = new WebSocket('ws://localhost:8081');

ws.onopen = function() {
    console.log('Connected to dashboard');
    
    // Subscribe to metrics updates
    ws.send(JSON.stringify({
        action: 'subscribe',
        channel: 'metrics'
    }));
};

ws.onmessage = function(event) {
    const message = JSON.parse(event.data);
    console.log('Received:', message);
};

ws.onerror = function(error) {
    console.error('WebSocket error:', error);
};

ws.onclose = function() {
    console.log('Disconnected');
};
```

### Message Types

#### Subscribe to Channel

```json
{
    "action": "subscribe",
    "channel": "metrics"
}
```

**Available Channels:**
- `metrics` - Periodic metrics updates
- `topics` - Topic discovery updates
- `nodes` - Node discovery updates
- `alerts` - Alert notifications
- `events` - General events

#### Unsubscribe from Channel

```json
{
    "action": "unsubscribe",
    "channel": "metrics"
}
```

#### Broadcast Message

The server broadcasts updates to all subscribed clients:

```json
{
    "type": "metrics_update",
    "timestamp": 1234567890123,
    "data": {
        "cpu_percent": 45.2,
        "memory_mb": 1024.5,
        "temperature_c": 65.5
    }
}
```

#### Topic Discovery Update

```json
{
    "type": "topic_discovered",
    "timestamp": 1234567890123,
    "data": {
        "name": "/sensor/camera",
        "type": "sensor_msgs/msg/Image"
    }
}
```

#### Alert Notification

```json
{
    "type": "alert",
    "timestamp": 1234567890123,
    "severity": "warning",
    "message": "CPU usage above threshold",
    "data": {
        "cpu_percent": 85.0,
        "threshold": 80.0
    }
}
```

---

## Usage Examples

### cURL Examples

**Get all topics:**
```bash
curl http://localhost:8080/api/topics
```

**Filter topics:**
```bash
curl "http://localhost:8080/api/topics?filter=sensor&sort=name"
```

**Get metrics history:**
```bash
curl "http://localhost:8080/api/metrics/history?duration_minutes=120"
```

**Call service:**
```bash
curl -X POST http://localhost:8080/api/services/my_service/call \
  -H "Content-Type: application/json" \
  -d '{"request": {"value": 42}}'
```

### Python Examples

```python
import requests
import websocket
import json

# REST API example
response = requests.get('http://localhost:8080/api/topics')
topics = response.json()['topics']

# WebSocket example
ws = websocket.create_connection('ws://localhost:8081')

ws.send(json.dumps({
    'action': 'subscribe',
    'channel': 'metrics'
}))

while True:
    message = ws.recv()
    print(json.loads(message))

ws.close()
```

### JavaScript/TypeScript Example

```typescript
// Fetch topics
const response = await fetch('http://localhost:8080/api/topics');
const data = await response.json();
console.log('Topics:', data.topics);

// WebSocket real-time updates
const ws = new WebSocket('ws://localhost:8081');

ws.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.type === 'metrics_update') {
        updateMetricsDisplay(message.data);
    }
};
```

---

## Error Codes

| Code | Meaning | Solution |
|------|---------|----------|
| 200 | OK | Request successful |
| 201 | Created | Resource created |
| 400 | Bad Request | Check request format |
| 401 | Unauthorized | Check authentication token |
| 404 | Not Found | Resource doesn't exist |
| 500 | Server Error | Check dashboard logs |

---

## Rate Limiting

The API implements rate limiting to prevent abuse:

- **10,000 requests** per minute per IP
- **WebSocket**: Maximum 1 message per second per client

Rate limit info in response headers:
```
X-RateLimit-Limit: 10000
X-RateLimit-Remaining: 9999
X-RateLimit-Reset: 1234567890
```

---

## CORS Support

Cross-Origin Resource Sharing (CORS) is enabled by default:

```
Access-Control-Allow-Origin: *
Access-Control-Allow-Methods: GET, POST, PUT, DELETE, OPTIONS
Access-Control-Allow-Headers: Content-Type, Authorization
```

For security, consider restricting origins in production:

```
Access-Control-Allow-Origin: https://mydomain.com
```

---

## Security Considerations

1. **HTTPS in Production** - Always use HTTPS/WSS in production
2. **Authentication** - Implement token-based auth for sensitive endpoints
3. **Rate Limiting** - Enabled by default
4. **Input Validation** - All inputs are validated
5. **CORS** - Restrict origins in production
6. **Firewall Rules** - Bind to localhost by default, expose via reverse proxy

### Sample nginx Configuration

```nginx
upstream dashboard {
    server localhost:8080;
}

upstream dashboard_ws {
    server localhost:8081;
}

server {
    listen 443 ssl http2;
    server_name dashboard.example.com;
    
    ssl_certificate /path/to/cert.pem;
    ssl_certificate_key /path/to/key.pem;
    
    # REST API
    location /api/ {
        proxy_pass http://dashboard;
        proxy_set_header Authorization $http_authorization;
    }
    
    # WebSocket
    location /ws {
        proxy_pass ws://dashboard_ws;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }
}
```

---

## Performance Metrics

- **API Latency**: < 50ms for typical requests
- **WebSocket Throughput**: Up to 10,000 messages/second
- **Connection Capacity**: Up to 1,000 concurrent WebSocket connections
- **Memory Overhead**: ~1MB per WebSocket connection

---

## Troubleshooting

### Cannot Connect to API

1. Check dashboard is running: `ps aux | grep ros2_dashboard`
2. Check port is available: `netstat -tuln | grep 8080`
3. Check firewall rules
4. Check logs: `tail -f ~/.ros2_dashboard/log`

### WebSocket Connection Fails

1. Check WebSocket port: `netstat -tuln | grep 8081`
2. Check reverse proxy (if using one) has WebSocket support
3. Verify `Connection: Upgrade` header is sent

### High Latency

1. Check CPU usage: `top`
2. Check network latency: `ping localhost`
3. Reduce update frequency in API calls
4. Use pagination for large datasets

---

## API Versioning

The API uses semantic versioning. Current version: `2.0.0`

Version available at: `GET /api/status`

Breaking changes will require major version increment.

---

## See Also

- [Plugin Architecture](PLUGIN_ARCHITECTURE.md) - Extend dashboard with plugins
- [Developer Guide](DEVELOPER.md) - Detailed development information
- [User Manual](USER_MANUAL.md) - End-user documentation
