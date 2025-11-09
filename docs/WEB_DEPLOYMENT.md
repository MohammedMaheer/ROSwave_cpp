# Web UI Deployment Guide

## Overview

The ROS2 Dashboard includes a modern, responsive web interface for remote access. This guide covers deployment options and configuration.

## Quick Start (Development)

### 1. Start the Dashboard

```bash
ros2_dashboard --api-port 8080 --ws-port 8081
```

### 2. Open Web Interface

```bash
# Option 1: Direct file access
firefox file:///path/to/web/index.html

# Option 2: With simple Python server
cd web/
python3 -m http.server 8000
# Then open http://localhost:8000
```

### 3. Configure Connection

1. Click ⚙️ in top right
2. Set API URL: `http://localhost:8080/api`
3. Set WebSocket URL: `ws://localhost:8081`
4. Click "Save Settings"

## Production Deployment

### Option 1: Docker

#### Create Dockerfile

```dockerfile
FROM node:18-alpine AS web-build
# Optional: build web assets
FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy dashboard binary
COPY ros2_dashboard ./
COPY web/ ./web/

# Expose ports
EXPOSE 8080 8081 3000

CMD ["./ros2_dashboard", "--api-port", "8080", "--ws-port", "8081"]
```

#### Build and Run

```bash
docker build -t ros2-dashboard .

# Run container
docker run -it \
  -p 8080:8080 \
  -p 8081:8081 \
  -p 3000:3000 \
  -v /dev:/dev \
  --network host \
  ros2-dashboard
```

### Option 2: Nginx Reverse Proxy

#### Nginx Configuration

```nginx
upstream dashboard_api {
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
    ssl_protocols TLSv1.2 TLSv1.3;
    
    # Gzip compression
    gzip on;
    gzip_types text/plain text/css application/json application/javascript;
    
    # Web UI static files
    location / {
        root /var/www/ros2-dashboard/web;
        try_files $uri $uri/ =404;
        add_header Cache-Control "max-age=3600";
    }
    
    # API proxy
    location /api/ {
        proxy_pass http://dashboard_api;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        
        # Timeouts
        proxy_connect_timeout 60s;
        proxy_send_timeout 60s;
        proxy_read_timeout 60s;
    }
    
    # WebSocket proxy
    location /ws {
        proxy_pass http://dashboard_ws;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        
        # WebSocket timeouts (long-lived)
        proxy_read_timeout 3600s;
        proxy_send_timeout 3600s;
    }
    
    # Health check endpoint
    location /health {
        proxy_pass http://dashboard_api/api/health;
        access_log off;
    }
}

# HTTP to HTTPS redirect
server {
    listen 80;
    server_name dashboard.example.com;
    return 301 https://$server_name$request_uri;
}
```

#### Deploy

```bash
# Copy web files
sudo mkdir -p /var/www/ros2-dashboard
sudo cp -r web/ /var/www/ros2-dashboard/

# Copy nginx config
sudo cp nginx.conf /etc/nginx/sites-available/ros2-dashboard
sudo ln -s /etc/nginx/sites-available/ros2-dashboard /etc/nginx/sites-enabled/

# Validate and reload
sudo nginx -t
sudo systemctl reload nginx
```

### Option 3: Apache with Proxy

```apache
<VirtualHost *:443>
    ServerName dashboard.example.com
    
    SSLEngine on
    SSLCertificateFile /path/to/cert.pem
    SSLCertificateKeyFile /path/to/key.pem
    
    # Enable proxy modules
    ProxyRequests On
    ProxyPreserveHost On
    
    # Web UI
    DocumentRoot /var/www/ros2-dashboard/web
    <Directory /var/www/ros2-dashboard/web>
        Require all granted
    </Directory>
    
    # API proxy
    ProxyPass /api/ http://localhost:8080/api/ timeout=300
    ProxyPassReverse /api/ http://localhost:8080/api/
    
    # WebSocket proxy
    ProxyPass /ws ws://localhost:8081/
    ProxyPassReverse /ws ws://localhost:8081/
    
    # Enable mod_rewrite for SPA
    <IfModule mod_rewrite.c>
        RewriteEngine On
        RewriteBase /
        RewriteRule ^index\.html$ - [L]
        RewriteCond %{REQUEST_FILENAME} !-f
        RewriteCond %{REQUEST_FILENAME} !-d
        RewriteRule . /index.html [L]
    </IfModule>
</VirtualHost>

# HTTP to HTTPS
<VirtualHost *:80>
    ServerName dashboard.example.com
    Redirect / https://dashboard.example.com/
</VirtualHost>
```

## Advanced Configuration

### Authentication

#### Using Nginx Auth Module

```nginx
location /api/ {
    auth_request /auth;
    proxy_pass http://dashboard_api;
}

location = /auth {
    proxy_pass http://auth-server:8000/check;
    proxy_pass_request_body off;
    proxy_set_header Content-Length "";
}
```

#### Using OAuth2-Proxy

```bash
docker run -it \
  --name oauth2-proxy \
  -p 4180:4180 \
  -e OAUTH2_PROXY_PROVIDER=google \
  -e OAUTH2_PROXY_CLIENT_ID=xxx \
  -e OAUTH2_PROXY_CLIENT_SECRET=xxx \
  -e OAUTH2_PROXY_COOKIE_SECRET=xxx \
  -e OAUTH2_PROXY_UPSTREAM=http://localhost:8080 \
  oauth2-proxy/oauth2-proxy:v7.4.0
```

### SSL/TLS Certificates

#### Let's Encrypt with Certbot

```bash
sudo apt-get install certbot python3-certbot-nginx

# Automatic setup
sudo certbot --nginx -d dashboard.example.com

# Manual renewal
sudo certbot renew --dry-run
```

#### Self-Signed Certificate (Development)

```bash
openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes
```

### Load Balancing

```nginx
upstream dashboard_cluster {
    least_conn;
    server dashboard1:8080 weight=1;
    server dashboard2:8080 weight=1;
    server dashboard3:8080 weight=1;
    
    keepalive 32;
}

server {
    location /api/ {
        proxy_pass http://dashboard_cluster;
        proxy_http_version 1.1;
        proxy_set_header Connection "";
    }
}
```

## Monitoring

### Health Check

```bash
# Check API
curl https://dashboard.example.com/api/health

# Check WebSocket
wscat -c wss://dashboard.example.com/ws
```

### Logging

```nginx
# Access log
access_log /var/log/nginx/ros2-dashboard-access.log combined;

# Error log
error_log /var/log/nginx/ros2-dashboard-error.log warn;

# Log format with timing
log_format timed_combined '$remote_addr - $remote_user [$time_local] '
                          '"$request" $status $body_bytes_sent '
                          '"$http_referer" "$http_user_agent" '
                          'rt=$request_time uct="$upstream_connect_time" '
                          'uht="$upstream_header_time" urt="$upstream_response_time"';
```

### Monitoring Stack

```yaml
# docker-compose.yml for monitoring
version: '3.8'
services:
  dashboard:
    image: ros2-dashboard:latest
    ports:
      - "8080:8080"
      - "8081:8081"
    
  prometheus:
    image: prom/prometheus:latest
    volumes:
      - ./prometheus.yml:/etc/prometheus/prometheus.yml
    ports:
      - "9090:9090"
    
  grafana:
    image: grafana/grafana:latest
    environment:
      GF_SECURITY_ADMIN_PASSWORD: admin
    ports:
      - "3000:3000"
    depends_on:
      - prometheus
```

## Performance Tuning

### Frontend Optimization

```javascript
// Enable service worker for offline support
if ('serviceWorker' in navigator) {
    navigator.serviceWorker.register('/sw.js');
}

// Use IndexedDB for caching
const db = new Dexie('DashboardDB');
db.version(1).stores({
    metrics: '++id, timestamp',
    topics: 'name'
});
```

### Backend Optimization

```nginx
# Gzip compression
gzip on;
gzip_min_length 1024;
gzip_types text/plain text/css application/json application/javascript text/xml application/xml+rss;
gzip_vary on;

# Caching
proxy_cache_path /var/cache/nginx levels=1:2 keys_zone=api_cache:10m max_size=100m inactive=60m;

location /api/metrics {
    proxy_cache api_cache;
    proxy_cache_valid 200 2s;
    proxy_pass http://dashboard_api;
}
```

## Troubleshooting

### Cannot Connect to API

```bash
# Check services
netstat -tuln | grep -E '8080|8081'

# Check firewall
sudo ufw status
sudo ufw allow 8080/tcp
sudo ufw allow 8081/tcp

# Test connection
curl http://localhost:8080/api/status
```

### WebSocket Connection Fails

```bash
# Check nginx logs
tail -f /var/log/nginx/error.log

# Verify WebSocket upgrade headers
curl -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" \
  http://localhost:8081/

# Check with wscat
npm install -g wscat
wscat -c ws://localhost:8081
```

### High Latency

```bash
# Monitor network
netstat -s

# Check proxy buffering (nginx)
proxy_buffering off;  # For real-time data

# Check API response times
time curl http://localhost:8080/api/metrics
```

## Scaling

### Horizontal Scaling

1. **Multiple Dashboard Instances**
   - Run multiple dashboard instances on different ports
   - Use load balancer (nginx, HAProxy)
   - Share configuration via shared volume/database

2. **Database Backend**
   - Store metrics in InfluxDB/TimescaleDB
   - Use Redis for caching
   - Implement data aggregation

### Vertical Scaling

```bash
# Increase file descriptors
ulimit -n 100000

# Increase TCP backlog
sysctl -w net.ipv4.tcp_max_syn_backlog=2048

# Memory tuning
sysctl -w net.core.rmem_max=134217728
sysctl -w net.core.wmem_max=134217728
```

## Security Checklist

- [ ] Enable HTTPS/TLS
- [ ] Configure CORS properly
- [ ] Implement authentication
- [ ] Use strong API tokens
- [ ] Enable rate limiting
- [ ] Set security headers:
  ```nginx
  add_header X-Frame-Options "SAMEORIGIN" always;
  add_header X-Content-Type-Options "nosniff" always;
  add_header X-XSS-Protection "1; mode=block" always;
  add_header Referrer-Policy "no-referrer-when-downgrade" always;
  ```
- [ ] Regular security updates
- [ ] Monitor access logs
- [ ] Backup configuration

## Deployment Checklist

- [ ] Test in staging environment
- [ ] Configure SSL certificates
- [ ] Set up reverse proxy
- [ ] Configure firewall rules
- [ ] Enable monitoring/logging
- [ ] Set up backups
- [ ] Document access procedures
- [ ] Test failover
- [ ] Create runbooks
- [ ] Schedule security updates

## See Also

- [REST API Documentation](REST_WEBSOCKET_API.md)
- [Developer Guide](DEVELOPER.md)
- [User Manual](USER_MANUAL.md)
