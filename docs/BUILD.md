# ROS2 Recording & Monitoring Dashboard - Build & Installation Guide

## System Requirements

### Operating System
- Ubuntu 20.04 LTS or later
- macOS 11+ (optional)
- Windows with WSL2 (optional)

### Compiler & Build Tools
- GCC 9+ or Clang 10+
- CMake 3.16+
- Git

### ROS2
- ROS2 Humble, Iron, or Jazzy
- rosbag2 and related packages

### Development Libraries
```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    qt5-qmake \
    qt5-default \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    sqlite3 \
    libsqlite3-dev \
    libcurl4-openssl-dev \
    libarchive-dev \
    nlohmann-json3-dev
```

### ROS2 Development
```bash
sudo apt-get install -y \
    ros-<distro>-rclcpp \
    ros-<distro>-rosbag2 \
    ros-<distro>-rosbag2-cpp
```

## Building the Project

### 1. Clone the Repository
```bash
git clone <repository-url>
cd cpp_ros2_live_status_dashboard
```

### 2. Create Build Directory
```bash
mkdir -p build
cd build
```

### 3. Configure with CMake
```bash
# Source ROS2 environment
source /opt/ros/<distro>/setup.bash

# Configure build
cmake -DCMAKE_BUILD_TYPE=Release ..
```

### 4. Build
```bash
# Build all targets
make -j$(nproc)

# Or build specific targets
make ros2_dashboard
make ros2_upload_server
make ros2_verify_performance
```

### 5. Install
```bash
# Install binaries and config files
sudo make install

# Or install to custom location
make install DESTDIR=/custom/path
```

## Running the Application

### Main Dashboard
```bash
# From build directory
./ros2_dashboard

# Or from installation path
ros2_dashboard
```

### Upload Server
```bash
# Start server on default port 8080
./ros2_upload_server

# Custom port and storage directory
./ros2_upload_server --port 9000 --storage /var/uploads
```

### Performance Verification
```bash
# Run benchmarks
./ros2_verify_performance

# Export results to JSON
./ros2_verify_performance --export --output results.json
```

## Configuration

### Edit Configuration File
```bash
# Edit config file
nano config/config.json

# Or use GUI settings dialog in application
```

### Key Configuration Options

**Cache Settings:**
- `cache.ttl_seconds`: 1-60 seconds (default: 5)

**Update Intervals:**
- `update_intervals.topics_ms`: Topic refresh interval (default: 3000ms)
- `update_intervals.metrics_ms`: Metrics refresh interval (default: 1000ms)

**Recording:**
- `recording.default_directory`: Where bags are saved
- `recording.compression_format`: zstd, lz4, or none

**Network:**
- `network.upload_endpoint`: Cloud server URL
- `network.chunk_size_mb`: Upload chunk size (default: 5MB)
- `network.max_concurrent_uploads`: Parallel uploads (default: 2)

## Troubleshooting

### Qt Libraries Not Found
```bash
# Install Qt development files
sudo apt-get install -y qt5-default libqt5widgets5 libqt5gui5 libqt5core5a

# Or add to CMake configuration
cmake -DQt5_DIR=/path/to/Qt5 ..
```

### ROS2 Not Found
```bash
# Ensure ROS2 is sourced before building
source /opt/ros/<distro>/setup.bash

# Check ROS2 environment
printenv ROS_DISTRO
```

### Build Errors
```bash
# Clean build
rm -rf build
mkdir build
cd build

# Verbose build for debugging
cmake -DCMAKE_VERBOSE_MAKEFILE=ON ..
make VERBOSE=1
```

## Testing

### Run Unit Tests
```bash
cd build
ctest --output-on-failure
```

### Performance Verification
```bash
./ros2_verify_performance
```

### Manual Testing Checklist
- [ ] Start dashboard successfully
- [ ] Discover ROS2 topics/nodes/services
- [ ] Start bag recording
- [ ] View metrics dashboard
- [ ] Export data
- [ ] Queue uploads
- [ ] Start upload server

## Development Setup

### IDE Configuration

**VS Code**
```bash
# Install extensions
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.cmake-tools

# Create .vscode/settings.json
{
    "cmake.generator": "Unix Makefiles",
    "cmake.buildDirectory": "${workspaceFolder}/build",
    "cmake.configureArgs": ["-DCMAKE_BUILD_TYPE=Debug"]
}
```

**CLion**
- Open project root folder
- CMake will be auto-detected
- Select build directory: `build`

## Performance Tuning

### Optimize Build
```bash
cmake -DCMAKE_BUILD_TYPE=Release -O3 ..
```

### Reduce Memory Usage
- Decrease `cache.ttl_seconds` (faster cleanup)
- Reduce `max_history_samples` in metrics collector
- Disable debug logging in settings

### Improve Responsiveness
- Increase `thread_pool.num_threads` (up to 4)
- Adjust UI refresh intervals based on needs

## Docker Deployment

### Build Docker Image
```bash
docker build -t ros2_dashboard .
```

### Run in Container
```bash
docker run -it \
    --network host \
    -e ROS_MASTER_URI=http://localhost:11311 \
    ros2_dashboard
```

See `Dockerfile` for detailed configuration.

## Uninstall

```bash
sudo make uninstall
```

## Getting Help

- Check documentation in `docs/` directory
- Review API documentation with Doxygen
- Search GitHub issues
- Create new issue with detailed error information
