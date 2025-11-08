# ROS2 Recording & Monitoring Dashboard - User Manual

## Table of Contents
1. [Overview](#overview)
2. [Getting Started](#getting-started)
3. [Main Features](#main-features)
4. [Tabs Overview](#tabs-overview)
5. [Recording Management](#recording-management)
6. [Data Export](#data-export)
7. [Cloud Integration](#cloud-integration)
8. [Metrics & Monitoring](#metrics--monitoring)
9. [Settings & Configuration](#settings--configuration)
10. [Troubleshooting](#troubleshooting)

## Overview

The ROS2 Recording & Monitoring Dashboard is a comprehensive desktop application for managing ROS2 robotics systems. It provides real-time monitoring, bag recording management, and cloud integration capabilities.

### Key Features
- **ROS2 System Discovery**: Real-time discovery of topics, nodes, and services
- **Bag Recording**: Manage rosbag2 recordings with flexible options
- **Live Metrics**: Monitor system, recording, and network performance
- **Data Export**: Package and export datasets for machine learning
- **Cloud Upload**: Queue and manage bag uploads to cloud storage
- **Offline-First**: Works seamlessly with or without network connectivity

## Getting Started

### Launching the Application
```bash
ros2_dashboard
```

### First-Time Setup
1. **Configure Upload Endpoint** (Optional)
   - Go to Settings → Network
   - Enter your cloud server URL and API credentials
   
2. **Set Recording Directory**
   - Go to Settings → Recording
   - Choose default directory for bag files

3. **Adjust Refresh Intervals**
   - Go to Settings → Update Intervals
   - Tune based on your system's performance needs

## Main Features

### Menu Bar

**File Menu**
- Exit: Quit the application

**View Menu**
- Refresh All (Ctrl+R): Force refresh all tabs

**Edit Menu**
- Settings (Ctrl+,): Open settings dialog

**Help Menu**
- About: Show application information

### Tool Bar
- **Refresh**: Force refresh all data
- **Settings**: Open settings dialog

### Status Bar
- **Status Label**: Current operation status
- **Connection Indicator**: Network connectivity (green = connected)

## Tabs Overview

### 1. Topics Tab
Displays all active ROS2 topics with detailed metadata.

**Features:**
- Topic name, message type, publisher/subscriber count
- Message frequency (Hz)
- Latest message preview

**Actions:**
- **Refresh**: Update topic list
- **Copy Name**: Copy selected topic name to clipboard
- **Click topic**: View latest message preview

**Performance:** Updates every 3 seconds

### 2. Nodes Tab
Displays all discovered ROS2 nodes and their connections.

**Features:**
- Node names and namespaces
- Subscriptions and publications
- Offered services

**Actions:**
- **Refresh**: Update node list

**Performance:** Updates every 3 seconds

### 3. Services Tab
Lists all available ROS2 services.

**Features:**
- Service names
- Service types
- Number of service servers

**Actions:**
- **Refresh**: Update service list

**Performance:** Updates every 3 seconds

### 4. Recording Tab
Manage ROS2 bag recordings.

**Recording Controls:**
1. **Select Output Directory**
   - Click "Browse..." to choose where bags are saved
   - Or use configured default directory

2. **Choose Compression Format**
   - zstd: Best compression (default)
   - lz4: Faster compression
   - none: No compression

3. **Select Topics (Optional)**
   - Leave empty to record all topics
   - Or specify specific topics

4. **Start/Stop Recording**
   - Click "Start Recording" to begin
   - Click "Stop Recording" to finish
   - Status shows: Active status, elapsed time, file size

**Bag Management:**
- **Refresh**: Reload list of recorded bags
- **Open in File Manager**: Browse to bag location
- **Play in RViz**: Launch RViz with selected bag (if installed)

**Status Indicators:**
- Recording status (Recording/Idle)
- Elapsed time (HH:MM:SS)
- Current file size (MB)

### 5. Metrics Tab
Real-time system and application performance monitoring.

**System Metrics:**
- CPU usage (%)
- Memory usage (MB)
- Disk space (free/total MB)
- Thermal status (if available)

**Recording Metrics:**
- Write speed (MB/s)
- Message throughput (msg/s)
- Compression ratio
- Total bytes written

**Application Metrics:**
- UI frame rate (FPS)
- Cache hit ratio (%)

**Actions:**
- **Export CSV**: Save metrics history to CSV file
- **Export JSON**: Save metrics history to JSON file
- **Refresh**: Update current metrics

### 6. Export Tab
Package recorded bags for machine learning datasets.

**Workflow:**
1. **Add Bags**
   - Click "Add Bags"
   - Select one or more bag files
   - Click "Remove" to remove selected

2. **Configure Export**
   - Set output directory
   - Add optional metadata annotations
   - Select specific topics (optional)

3. **Export**
   - Click "Export"
   - Monitor progress bar
   - Files will be compressed to .tar.gz

**Export Contents:**
- Raw rosbag2 files
- `metadata.json` (recording info)
- `schema.json` (message definitions)
- `.tar.gz` compressed archive

### 7. Network Tab
Discover and monitor robots on network (multicast discovery).

**Features:**
- Discover robots on LAN
- View robot status
- Last seen timestamp
- Network connectivity indicator

**Actions:**
- **Discover Robots**: Scan for robots
- **Refresh**: Update robot status

### 8. Upload Tab
Manage cloud uploads and queue.

**Upload Queue:**
- File name
- Current status (Pending, Uploading, Completed, Failed)
- Progress percentage
- File size (MB)
- Upload priority

**Actions:**
- **Retry**: Retry failed upload
- **Cancel**: Cancel pending/active upload
- **Clear Completed**: Remove completed uploads from queue
- **Refresh**: Update queue display

**Upload Statistics:**
- Total queued files
- Pending uploads
- Active uploads
- Total queue size (MB)

## Recording Management

### Starting a Recording

1. Go to **Recording** tab
2. Select output directory (or use default)
3. Choose compression format (recommended: zstd)
4. Click **"Start Recording"**
5. Status updates to show: "Recording Active"

### Monitoring Recording

- **Elapsed Time**: Shows recording duration
- **File Size**: Current bag file size
- **Write Speed**: MB/s being written (in Metrics tab)

### Stopping Recording

1. Click **"Stop Recording"** button
2. Recording status changes to "Idle"
3. Bag file is finalized

### Accessing Recorded Bags

1. Go to **Recording** tab
2. Select bag in list
3. Actions:
   - **Open in File Manager**: Browse to bag
   - **Play in RViz**: Launch RViz with bag (if installed)

## Data Export

### Exporting Bags for ML

1. Go to **Export** tab
2. Click **"Add Bags"** and select bag files
3. (Optional) Add metadata annotation
4. (Optional) Select specific topics
5. Click **"Export"**
6. Wait for progress to complete (100%)

### Export Output
Location: `{output_directory}/`
Contents:
```
export_dir/
├── bag_file_1.db3
├── bag_file_2.db3
├── metadata.json          # Recording metadata
├── schema.json            # Message type definitions
└── export_dir.tar.gz      # Compressed archive
```

### Metadata File Example
```json
{
  "export_version": "1.0",
  "export_date": "2025-01-15T10:30:00Z",
  "bag_files": ["recording_1.db3", "recording_2.db3"],
  "topics": ["/sensor/lidar", "/sensor/camera"],
  "message_counts": {
    "/sensor/lidar": 15000,
    "/sensor/camera": 30000
  },
  "total_duration": "00:10:23",
  "total_size_bytes": 5368709120
}
```

## Cloud Integration

### Configuring Cloud Upload

1. Go to **Edit** → **Settings**
2. Select **Network** tab
3. Configure:
   - **Upload Endpoint**: Cloud server URL (e.g., http://example.com:8080)
   - **API Key**: Your authentication key
   - **Chunk Size**: 5MB recommended
   - **Max Concurrent Uploads**: 2 recommended
   - **Bandwidth Limit**: 0 for unlimited

4. Click **Apply** → **OK**

### Uploading Bags

1. Locate bag file on disk
2. Go to **Uploads** tab
3. Click **"Add to Queue"** or drag-and-drop
4. Upload begins automatically if network available
5. Monitor progress in queue table

### Queue Management

**Upload Status:**
- **Pending**: Waiting to upload
- **Uploading**: Currently transferring
- **Completed**: Successfully uploaded
- **Failed**: Upload failed (retry available)

**Queue Priority:**
- Set 1-10 priority level (10 = highest)
- Higher priority uploads start first

**Retry Failed Uploads:**
1. Select failed upload
2. Click **"Retry"**
3. Upload attempts up to 3 times with backoff

### Offline-First Behavior

- Uploads are queued locally even without network
- Queue persists across application restarts
- Uploads resume automatically when network available
- Works seamlessly with or without cloud server

## Metrics & Monitoring

### Understanding Metrics

**System Metrics:**
- **CPU Usage**: Percentage of CPU currently in use
- **Memory Usage**: MB of RAM allocated
- **Disk Space**: Free and total disk capacity
- **Thermal**: CPU temperature (if available)

**Recording Metrics** (only during active recording):
- **Write Speed**: MB/s data throughput
- **Messages/Sec**: Message rate being recorded
- **Compression Ratio**: Size reduction from compression
- **Total Bytes**: Cumulative bytes written

**Application Metrics:**
- **Frame Rate**: UI refresh rate (FPS)
- **Cache Hit Ratio**: Percentage of cached data hits

### Exporting Metrics

**To CSV:**
1. Go to **Metrics** tab
2. Click **"Export CSV"**
3. Choose file location
4. File includes: timestamp, CPU, memory, write speed, frame rate, etc.

**To JSON:**
1. Click **"Export JSON"**
2. Choose file location
3. Structured JSON format with full history

### Metrics Refresh Interval
Default: 1000ms (1 second)
Adjustable in Settings → Update Intervals → Metrics Refresh

## Settings & Configuration

### Accessing Settings

**Method 1:** Menu
- Edit → Settings

**Method 2:** Tool Bar
- Click Settings button

**Method 3:** Keyboard
- Ctrl+Comma

### Settings Tabs

#### Cache
- **TTL (Time-To-Live)**: 1-60 seconds
  - Higher = better caching, staler data
  - Lower = fresher data, more API calls
  - Default: 5 seconds

#### Update Intervals
- **Topic Refresh**: How often to query topics (default: 3000ms)
- **Node Refresh**: How often to query nodes (default: 3000ms)
- **Metrics Refresh**: How often to update metrics (default: 1000ms)
- Range: 500-10000ms

#### Performance
- **Worker Threads**: Thread pool size (1-4, default: 2)
  - Higher = more concurrent tasks
  - Lower = less CPU, potentially slower

#### Network
- **Upload Endpoint**: Cloud server URL
- **API Key**: Authentication credential
- **Chunk Size**: Upload block size in MB (default: 5)
- **Max Concurrent**: Parallel uploads (default: 2)
- **Bandwidth Limit**: Max MB/s (0 = unlimited)

#### Recording
- **Default Directory**: Where new bags are saved

#### UI
- **Dark Theme**: Enable dark mode (experimental)
- **Debug Logging**: Enable verbose console logging

### Saving Settings

1. Adjust settings as desired
2. Click **"Apply"** to apply immediately
3. Click **"OK"** to apply and close
4. Settings are saved to config file

### Resetting Settings

1. Click **"Reset"** to restore defaults
2. Or manually edit `config.json`

## Troubleshooting

### Topics/Nodes Not Showing

**Problem:** Topics tab is empty

**Solutions:**
1. Ensure ROS2 is running: `ros2 topic list`
2. Click "Refresh" button
3. Check ROS2 environment: `echo $ROS_DOMAIN_ID`
4. Increase cache TTL in Settings

### Recording Fails to Start

**Problem:** "Failed to start recording"

**Solutions:**
1. Verify output directory exists and is writable
2. Check disk space: `df -h`
3. Ensure `ros2 bag` command works: `ros2 bag info`
4. Check permissions on output directory

### High CPU Usage

**Problem:** Application using too much CPU

**Solutions:**
1. Reduce update intervals in Settings
2. Decrease thread pool size
3. Increase cache TTL
4. Only refresh active tabs

### Network Uploads Not Working

**Problem:** "Network unavailable" error

**Solutions:**
1. Check internet connection: `ping 8.8.8.8`
2. Verify upload endpoint is correct in Settings
3. Test endpoint manually: `curl http://endpoint:port/health`
4. Check API key is valid
5. Review upload server logs

### Memory Usage High

**Problem:** Application using excessive memory

**Solutions:**
1. Clear metrics history: Settings → Clear History
2. Reduce cache TTL
3. Limit maximum history samples
4. Clear completed uploads from queue
5. Restart application

### GUI Freezes

**Problem:** UI becomes unresponsive

**Solutions:**
1. Wait for current operation to complete
2. Try clicking "Refresh All"
3. Close and reopen application
4. Check system resources: `top` or `htop`
5. Enable debug logging to find bottleneck

### Export Takes Too Long

**Problem:** Data export is slow

**Solutions:**
1. Reduce number of bags per export
2. Use faster compression format (lz4 instead of zstd)
3. Export to faster storage (SSD vs HDD)
4. Increase thread pool size in Settings

### Upload Server Connection Failed

**Problem:** "Cannot connect to upload server"

**Solutions:**
1. Ensure server is running: `ps aux | grep upload_server`
2. Check server port: `netstat -tuln | grep 8080`
3. Verify firewall allows connection: `sudo ufw allow 8080`
4. Test with curl: `curl http://localhost:8080/health`
5. Check server logs for errors

## Performance Tips

1. **Optimize for Your System**
   - Adjust update intervals based on ROS2 activity
   - Reduce cache TTL if memory is limited

2. **Reduce CPU Usage**
   - Only open needed tabs
   - Increase update intervals for inactive tabs
   - Use lighter compression format (none or lz4)

3. **Improve Responsiveness**
   - Increase thread pool size (up to 4)
   - Reduce cache TTL for instant updates
   - Disable debug logging

4. **Maximize Reliability**
   - Enable cloud uploads for data redundancy
   - Regularly clear completed uploads
   - Monitor disk space on recording drive

## Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| Ctrl+R | Refresh all data |
| Ctrl+, | Open settings |
| Ctrl+Q | Exit application |
| Tab | Switch between tabs |

## Getting Help

- Check documentation: `docs/` directory
- Review error messages in status bar
- Enable debug logging in Settings
- Check application logs (location depends on OS)
- Create GitHub issue with error details
