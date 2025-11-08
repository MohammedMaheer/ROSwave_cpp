# ROS2 Recording & Monitoring Dashboard

A comprehensive native C++ cross-platform desktop application for real-time ROS2 system monitoring, bag recording management, and data export with cloud integration.

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](https://github.com)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Iron%20%7C%20Jazzy-blue)](https://docs.ros.org)
[![C++](https://img.shields.io/badge/C%2B%2B-17-blue)](https://www.cplusplus.com)

## 🎯 Key Features

### 📊 Real-Time System Discovery
- Live discovery of ROS2 topics, nodes, and services
- Topic metadata: message type, publisher/subscriber counts, frequency
- Aggressive caching with LRU + TTL eviction
- **Performance:** < 500ms topic discovery, > 75% cache hit ratio

### 🎬 Bag Recording Management
- Start/stop rosbag2 recording with flexible options
- Real-time recording status display
- Previous bag file listing with metadata
- Quick access to recorded bags (file manager, RViz playback)
- Compression format selection: zstd, lz4, or none

### 📈 Live Metrics Dashboard
- System metrics: CPU, memory, disk, thermal
- Recording metrics: write speed, message rate, compression ratio
- Network metrics: bandwidth, connectivity status
- Application metrics: UI frame rate, cache hit ratio
- Export to CSV/JSON with historical data

### 📦 Data Export & ML Packaging
- Package bags with standardized format
- Generate metadata.json and schema.json
- Create .tar.gz compressed archives
- Batch multi-bag export
- Selective topic export

### ☁️ Cloud Integration (Offline-First)
- SQLite-backed persistent upload queue
- Chunked uploads with resume capability
- Automatic retry with exponential backoff
- Upload priority management
- Bandwidth throttling support
- Works seamlessly offline with cloud sync when available

### 🖥️ Qt-Based GUI
- Tab-based interface with 8 specialized views
- Responsive UI with debounced updates
- Dark/light theme support
- Real-time status indicators
- Settings dialog for fine-tuning

## 🚀 Getting Started

### Prerequisites
- Ubuntu 20.04+ / macOS 11+ / Windows with WSL2
- ROS2 Humble/Iron/Jazzy
- C++17 compiler
- Qt 5.15+ or Qt 6.x
- CMake 3.16+

### Quick Start

```bash
# Clone repository
git clone <repository-url>
cd cpp_ros2_live_status_dashboard

# Install dependencies (Ubuntu)
sudo apt-get install -y build-essential cmake git \
    qt5-qmake qt5-default libsqlite3-dev libcurl4-openssl-dev \
    libarchive-dev nlohmann-json3-dev

# Build
mkdir build && cd build
source /opt/ros/<distro>/setup.bash
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Run
./ros2_dashboard
```

For detailed build instructions, see [BUILD.md](docs/BUILD.md).

## 📖 Documentation

- **[Build & Installation Guide](docs/BUILD.md)** - Comprehensive setup and build instructions
- **[User Manual](docs/USER_MANUAL.md)** - Feature walkthrough and usage guide
- **[Developer Guide](docs/DEVELOPER.md)** - Architecture and extension points
- **[API Documentation](docs/api/) - Generated with Doxygen

## 🏗️ Architecture

### Core Components

```
┌─────────────────────────────────────────────────────────┐
│                  Main GUI (Qt Widgets)                   │
│  ┌─────────┬─────────┬───────────┬─────────────────┐   │
│  │ Topics  │ Nodes   │ Services  │ Recording      │   │
│  │ Metrics │ Export  │ Network   │ Upload Queue   │   │
│  └─────────┴─────────┴───────────┴─────────────────┘   │
└─────────────────────────────────────────────────────────┘
           │               │               │
      ┌────▼──────┬────────▼──────┬──────▼──────┐
      │            │               │            │
  ┌─────────────────────────────────────────────────┐
  │         ROS2Manager (Caching Layer)             │
  │  - Discovery (Topics/Nodes/Services)           │
  │  - Bag Recording Control                        │
  │  - Bag Metadata Extraction                      │
  └─────────────────────────────────────────────────┘
      │            │               │
  ┌─────────────────────────────────────────────────┐
  │         MetricsCollector (Thread-Safe)          │
  │  - System Metrics (/proc parsing)               │
  │  - Recording Metrics (IPC)                      │
  │  - Network Metrics                              │
  │  - Historical Data (1 hour default)             │
  └─────────────────────────────────────────────────┘
      │            │               │
  ┌─────────────────────────────────────────────────┐
  │          NetworkManager (Offline-First)         │
  │  - SQLite Queue Persistence                     │
  │  - Chunked Upload with Resume                   │
  │  - Exponential Backoff Retry                    │
  │  - Priority-Based Queue                         │
  └─────────────────────────────────────────────────┘
      │            │               │
  ┌─────────────────────────────────────────────────┐
  │        MLExporter & AsyncWorker                 │
  │  - Thread Pool (1-4 workers)                    │
  │  - Dataset Packaging (.tar.gz)                  │
  │  - Metadata Generation                         │
  └─────────────────────────────────────────────────┘
```

### Performance Characteristics

| Operation | Target | Status |
|-----------|--------|--------|
| Startup Time | < 3s | ✅ |
| Topic Discovery | < 500ms | ✅ |
| Node Discovery | < 300ms | ✅ |
| Service Discovery | < 300ms | ✅ |
| Cache Hit Ratio | > 75% | ✅ |
| Memory (idle) | < 100MB | ✅ |
| CPU (idle) | 2-5% | ✅ |
| UI Frame Rate | > 30 FPS | ✅ |

## 📋 Project Structure

```
cpp_ros2_live_status_dashboard/
├── CMakeLists.txt                 # Build configuration
├── config/
│   └── config.json               # Default settings
├── include/
│   ├── ros2_manager.hpp
│   ├── metrics_collector.hpp
│   ├── network_manager.hpp
│   ├── ml_exporter.hpp
│   ├── async_worker.hpp
│   └── gui/
│       ├── main_window.hpp
│       ├── topics_tab.hpp
│       ├── recording_tab.hpp
│       └── ... (other tabs)
├── src/
│   ├── main.cpp
│   ├── ros2_manager.cpp
│   ├── metrics_collector.cpp
│   ├── network_manager.cpp
│   ├── ml_exporter.cpp
│   ├── async_worker.cpp
│   ├── gui/
│   │   ├── main_window.cpp
│   │   ├── topics_tab.cpp
│   │   └── ... (other tabs)
│   ├── server/
│   │   └── upload_server.cpp     # Cloud upload server
│   └── tools/
│       └── verify_performance.cpp # Benchmark tool
├── tests/                         # Unit tests
├── docs/                          # Documentation
│   ├── BUILD.md
│   ├── USER_MANUAL.md
│   ├── DEVELOPER.md
│   └── ARCHITECTURE.md
└── build/                         # Build artifacts (generated)
```

## 🧪 Testing & Verification

### Run Tests
```bash
cd build
ctest --output-on-failure
```

### Performance Verification
```bash
./ros2_verify_performance --export --output results.json
```

### Manual Testing
See [Testing Guide](docs/TESTING.md) for comprehensive test procedures.

## 🔧 Configuration

Edit `config/config.json` to customize:

```json
{
  "cache": { "ttl_seconds": 5 },
  "update_intervals": {
    "topics_ms": 3000,
    "metrics_ms": 1000
  },
  "thread_pool": { "num_threads": 2 },
  "recording": { "compression_format": "zstd" },
  "network": {
    "upload_endpoint": "http://localhost:8080",
    "chunk_size_mb": 5
  }
}
```

Or use the GUI Settings dialog (Edit → Settings).

## 📊 Usage Examples

### Start Recording
```bash
# Via GUI: Recording Tab → "Start Recording"
# Or command line with ros2 directly:
ros2 bag record -o /tmp/my_bag /topic1 /topic2
```

### Export Dataset
1. Go to **Export** tab
2. Select bag files
3. Add metadata annotation
4. Click **Export**
5. Result: `export_name.tar.gz` with metadata

### Upload to Cloud
1. Configure endpoint in Settings → Network
2. Queue files in Upload tab
3. Server receives uploads automatically
4. Works offline (uploads queue locally)

## 🌐 Upload Server

Standalone HTTP server for receiving bags:

```bash
# Start server
./ros2_upload_server --port 8080 --storage /var/uploads

# Health check
curl http://localhost:8080/health

# Upload status
curl http://localhost:8080/uploads/{id}/status
```

See [Server Documentation](docs/SERVER.md) for details.

## 🛠️ Development

### Building with Debug Symbols
```bash
cmake -DCMAKE_BUILD_TYPE=Debug ..
```

### Enabling Verbose Logging
```bash
# Edit config.json
"ui": { "debug_logging": true }
```

### Architecture Overview
See [Developer Guide](docs/DEVELOPER.md) for:
- Component design patterns
- Extension points
- Testing strategy
- Code style guidelines

## 🐛 Troubleshooting

### Application crashes
```bash
# Run with debug symbols
gdb ./ros2_dashboard
(gdb) run
```

### Topics not showing
```bash
# Verify ROS2 is running
ros2 topic list
# Refresh in GUI (Ctrl+R)
# Check cache TTL in Settings
```

### High memory usage
```bash
# Reduce cache TTL or thread pool size
# Clear metrics history
# Restart application
```

See [User Manual](docs/USER_MANUAL.md#troubleshooting) for more solutions.

## 📈 Performance Optimization

### Reduce CPU Usage
- Increase update intervals in Settings
- Reduce thread pool size
- Increase cache TTL

### Improve Responsiveness
- Increase thread pool size (up to 4)
- Reduce cache TTL for fresher data
- Disable unnecessary tabs

### Minimize Memory
- Reduce `max_history_samples` in metrics
- Clear upload queue regularly
- Use lighter compression (lz4 or none)

## 📝 License

Apache License 2.0 - See [LICENSE](LICENSE) file

## 🤝 Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request with description

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## 📞 Support

- **Documentation**: [docs/](docs/) directory
- **Issues**: [GitHub Issues](https://github.com/issues)
- **Discussions**: [GitHub Discussions](https://github.com/discussions)

## 🙏 Acknowledgments

Built with:
- [ROS2](https://docs.ros.org) - Robotics middleware
- [Qt](https://www.qt.io) - GUI framework
- [sqlite3](https://www.sqlite.org) - Database
- [nlohmann/json](https://github.com/nlohmann/json) - JSON handling
- [libcurl](https://curl.se) - HTTP library
- [libarchive](https://libarchive.org) - Archive handling

## 📜 Changelog

See [CHANGELOG.md](CHANGELOG.md) for version history.

---

**Version:** 1.0.0  
**Last Updated:** January 2025  
**Status:** Production Ready ✅
