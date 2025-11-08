FROM ubuntu:22.04

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
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
    nlohmann-json3-dev \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 (Humble)
RUN curl -sSL https://repo.ros2.org/ros.key | apt-key add - && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu jammy main" > \
    /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rclcpp \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-cpp \
    ros-humble-rclcpp-lifecycle \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /workspace

# Copy project files
COPY . /workspace/

# Build the project
RUN mkdir -p build && cd build && \
    source /opt/ros/humble/setup.bash && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc)

# Install binaries
RUN cd build && make install

# Create non-root user
RUN useradd -m -s /bin/bash ros2user && \
    chown -R ros2user:ros2user /workspace

USER ros2user

# Setup ROS2 environment
ENV ROS_DISTRO=humble
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8080/health || exit 1

# Default command - start dashboard
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && ros2_dashboard"]

# Expose ports for upload server and communication
EXPOSE 8080 11311 5005 6006
