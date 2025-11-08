/**
 * @file ros2_manager.hpp
 * @brief ROS2 system discovery and management with caching
 * @author Dashboard Team
 */

#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <chrono>
#include <mutex>
#include <functional>
#include <optional>

namespace ros2_dashboard {

/**
 * @struct TopicInfo
 * @brief Information about a ROS2 topic
 */
struct TopicInfo {
    std::string name;
    std::string msg_type;
    int publisher_count = 0;
    int subscription_count = 0;
    double frequency_hz = 0.0;
    std::string latest_message;
};

/**
 * @struct NodeInfo
 * @brief Information about a ROS2 node
 */
struct NodeInfo {
    std::string name;
    std::string namespace_;
    std::vector<std::string> subscriptions;
    std::vector<std::string> publications;
    std::vector<std::string> services;
};

/**
 * @struct ServiceInfo
 * @brief Information about a ROS2 service
 */
struct ServiceInfo {
    std::string name;
    std::string service_type;
    std::vector<std::string> servers;
};

/**
 * @class ROS2Manager
 * @brief Manages ROS2 system discovery and bag recording
 * 
 * Implements aggressive caching with LRU + TTL eviction for efficient discovery.
 * All operations use fast-fail timeout strategy (1 second max).
 */
class ROS2Manager {
public:
    ROS2Manager();
    ~ROS2Manager();

    /**
     * @brief Discover all ROS2 topics
     * @return Vector of TopicInfo structs
     * @performance < 500ms
     */
    std::vector<TopicInfo> get_topics();

    /**
     * @brief Discover all ROS2 nodes
     * @return Vector of NodeInfo structs
     * @performance < 300ms
     */
    std::vector<NodeInfo> get_nodes();

    /**
     * @brief Discover all ROS2 services
     * @return Vector of ServiceInfo structs
     * @performance < 300ms
     */
    std::vector<ServiceInfo> get_services();

    /**
     * @brief Get latest message from a specific topic
     * @param topic_name Name of the topic
     * @return Latest message content as string
     */
    std::optional<std::string> get_topic_echo(const std::string& topic_name);

    /**
     * @brief Start recording a rosbag2
     * @param output_dir Output directory for the bag
     * @param topic_filter Topics to record (empty = all)
     * @param compression_format Compression format (none, zstd, lz4)
     * @return true if recording started successfully
     */
    bool start_recording(const std::string& output_dir,
                        const std::vector<std::string>& topic_filter = {},
                        const std::string& compression_format = "zstd");

    /**
     * @brief Stop active recording
     * @return true if recording stopped successfully
     */
    bool stop_recording();

    /**
     * @brief Check if recording is active
     * @return true if recording is in progress
     */
    bool is_recording() const;

    /**
     * @brief Get recording status information
     * @return Map with keys: "active", "elapsed_time", "file_size"
     */
    std::map<std::string, std::string> get_recording_status();

    /**
     * @struct BagMetadata
     * @brief Metadata extracted from a bag file
     */
    struct BagMetadata {
        std::string filename;
        std::string duration;
        int64_t file_size_bytes = 0;
        std::vector<std::string> topics;
        std::map<std::string, int64_t> message_counts;
        std::string created_time;
    };

    /**
     * @brief Extract metadata from a bag file
     * @param bag_path Path to the bag file
     * @return BagMetadata struct with extracted information
     */
    BagMetadata extract_bag_metadata(const std::string& bag_path);

    /**
     * @brief List all previously recorded bags
     * @param directory Directory containing bag files
     * @return Vector of BagMetadata
     */
    std::vector<BagMetadata> list_bags(const std::string& directory);

    /**
     * @brief Set cache TTL in seconds
     * @param ttl_seconds TTL value (1-60 seconds)
     */
    void set_cache_ttl(int ttl_seconds);

    /**
     * @brief Get current cache hit ratio
     * @return Cache hit ratio (0.0 - 1.0)
     */
    double get_cache_hit_ratio() const;

    /**
     * @brief Manually invalidate caches
     */
    void invalidate_cache();

private:
    /**
     * @struct CacheEntry
     * @brief Internal cache entry with TTL
     */
    template<typename T>
    struct CacheEntry {
        T data;
        std::chrono::steady_clock::time_point timestamp;
        bool is_valid(int ttl_seconds) const {
            auto age = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - timestamp).count();
            return age < ttl_seconds;
        }
    };

    mutable std::mutex cache_mutex_;
    int cache_ttl_seconds_ = 5;
    
    std::optional<CacheEntry<std::vector<TopicInfo>>> topics_cache_;
    std::optional<CacheEntry<std::vector<NodeInfo>>> nodes_cache_;
    std::optional<CacheEntry<std::vector<ServiceInfo>>> services_cache_;

    uint64_t cache_hits_ = 0;
    uint64_t cache_misses_ = 0;
    bool recording_active_ = false;
    std::string recording_process_id_;
    std::chrono::steady_clock::time_point recording_start_time_;
    std::string recording_output_dir_;

    // Internal methods
    std::vector<TopicInfo> discover_topics_();
    std::vector<NodeInfo> discover_nodes_();
    std::vector<ServiceInfo> discover_services_();
};

}  // namespace ros2_dashboard
