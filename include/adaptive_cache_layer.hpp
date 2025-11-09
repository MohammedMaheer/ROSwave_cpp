#pragma once

#include <map>
#include <queue>
#include <memory>
#include <chrono>
#include <mutex>
#include <functional>
#include <optional>
#include <string>
#include <vector>

/**
 * @brief Adaptive cache manager with intelligent TTL and LRU eviction
 * 
 * Features:
 * - Adaptive TTL based on access patterns
 * - LRU eviction when capacity reached
 * - Thread-safe operations
 * - Size and count limits
 * - Automatic cleanup
 * - Statistics tracking
 */
template<typename Key, typename Value>
class AdaptiveCacheManager {
public:
    struct CacheEntry {
        Value data;
        std::chrono::system_clock::time_point created_at;
        std::chrono::system_clock::time_point last_accessed;
        size_t access_count = 0;
        std::chrono::milliseconds ttl;
        size_t size_bytes = 0;  // Track actual size
    };

    struct CacheStats {
        size_t hits = 0;
        size_t misses = 0;
        size_t evictions = 0;
        size_t current_size = 0;
        double hit_rate = 0.0;
    };

    explicit AdaptiveCacheManager(size_t max_entries = 1000,
                                 size_t max_bytes = 100 * 1024 * 1024,
                                 std::chrono::milliseconds base_ttl = std::chrono::seconds(30))
        : max_entries_(max_entries), max_bytes_(max_bytes), base_ttl_(base_ttl),
          current_bytes_(0) {}

    /**
     * @brief Get value from cache
     */
    std::optional<Value> get(const Key& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto it = cache_.find(key);
        if (it == cache_.end()) {
            stats_.misses++;
            return std::nullopt;
        }

        // Check if expired
        auto now = std::chrono::system_clock::now();
        if (now - it->second.created_at > it->second.ttl) {
            cache_.erase(it);
            stats_.misses++;
            return std::nullopt;
        }

        // Update access statistics
        it->second.last_accessed = now;
        it->second.access_count++;
        stats_.hits++;
        
        update_hit_rate();
        return it->second.data;
    }

    /**
     * @brief Put value in cache
     */
    void put(const Key& key, const Value& value, size_t size_bytes) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Remove old entry if exists
        auto it = cache_.find(key);
        if (it != cache_.end()) {
            current_bytes_ -= it->second.size_bytes;
        }

        // Evict if necessary
        while (cache_.size() >= max_entries_ || current_bytes_ + size_bytes > max_bytes_) {
            if (cache_.empty()) break;
            evict_lru();
        }

        // Calculate adaptive TTL based on access patterns
        auto adaptive_ttl = calculate_adaptive_ttl();

        // Insert new entry
        auto now = std::chrono::system_clock::now();
        cache_[key] = {
            value,
            now,
            now,
            0,
            adaptive_ttl,
            size_bytes
        };
        
        current_bytes_ += size_bytes;
    }

    /**
     * @brief Remove specific entry
     */
    void remove(const Key& key) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto it = cache_.find(key);
        if (it != cache_.end()) {
            current_bytes_ -= it->second.size_bytes;
            cache_.erase(it);
        }
    }

    /**
     * @brief Clear entire cache
     */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        cache_.clear();
        current_bytes_ = 0;
    }

    /**
     * @brief Cleanup expired entries
     */
    void cleanup_expired() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto now = std::chrono::system_clock::now();
        auto it = cache_.begin();
        
        while (it != cache_.end()) {
            if (now - it->second.created_at > it->second.ttl) {
                current_bytes_ -= it->second.size_bytes;
                it = cache_.erase(it);
                stats_.evictions++;
            } else {
                ++it;
            }
        }
    }

    /**
     * @brief Get cache statistics
     */
    CacheStats get_stats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stats_;
    }

    /**
     * @brief Set base TTL
     */
    void set_base_ttl(std::chrono::milliseconds ttl) {
        base_ttl_ = ttl;
    }

    /**
     * @brief Get current cache size in bytes
     */
    size_t get_current_bytes() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_bytes_;
    }

    /**
     * @brief Get number of entries
     */
    size_t get_entry_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cache_.size();
    }

private:
    void evict_lru() {
        // Find least recently used entry
        auto lru_it = cache_.begin();
        for (auto it = cache_.begin(); it != cache_.end(); ++it) {
            if (it->second.last_accessed < lru_it->second.last_accessed) {
                lru_it = it;
            }
        }
        
        current_bytes_ -= lru_it->second.size_bytes;
        cache_.erase(lru_it);
        stats_.evictions++;
    }

    std::chrono::milliseconds calculate_adaptive_ttl() const {
        // Increase TTL based on overall hit rate
        double factor = 1.0 + (stats_.hit_rate * 2.0); // 1x to 3x multiplier
        return std::chrono::milliseconds(
            static_cast<long>(base_ttl_.count() * factor)
        );
    }

    void update_hit_rate() {
        size_t total = stats_.hits + stats_.misses;
        if (total > 0) {
            stats_.hit_rate = static_cast<double>(stats_.hits) / total;
        }
    }

    mutable std::mutex mutex_;
    std::map<Key, CacheEntry> cache_;
    size_t max_entries_;
    size_t max_bytes_;
    std::chrono::milliseconds base_ttl_;
    size_t current_bytes_;
    mutable CacheStats stats_;
};

/**
 * @brief ROS2-specific adaptive cache for topics
 */
class ROS2TopicCache {
public:
    struct TopicCacheEntry {
        std::string name;
        std::string msg_type;
        int publisher_count;
        int subscription_count;
        double frequency_hz;
        std::string latest_message;
    };

    explicit ROS2TopicCache(size_t max_topics = 500)
        : cache_(max_topics, 50 * 1024 * 1024, std::chrono::seconds(60)) {}

    /**
     * @brief Get cached topic info
     */
    std::optional<TopicCacheEntry> get_topic(const std::string& topic_name) {
        return cache_.get(topic_name);
    }

    /**
     * @brief Cache topic info
     */
    void cache_topic(const TopicCacheEntry& entry) {
        cache_.put(entry.name, entry, sizeof(TopicCacheEntry) + entry.name.size() +
                                      entry.msg_type.size() + entry.latest_message.size());
    }

    /**
     * @brief Invalidate specific topic cache
     */
    void invalidate_topic(const std::string& topic_name) {
        cache_.remove(topic_name);
    }

    /**
     * @brief Clear all topic cache
     */
    void clear_all() {
        cache_.clear();
    }

    /**
     * @brief Get cache hit rate
     */
    double get_hit_rate() const {
        return cache_.get_stats().hit_rate;
    }

    /**
     * @brief Cleanup expired entries
     */
    void cleanup() {
        cache_.cleanup_expired();
    }

private:
    AdaptiveCacheManager<std::string, TopicCacheEntry> cache_;
};

/**
 * @brief ROS2-specific adaptive cache for nodes
 */
class ROS2NodeCache {
public:
    struct NodeCacheEntry {
        std::string name;
        std::string namespace_;
        std::vector<std::string> subscriptions;
        std::vector<std::string> publications;
        std::vector<std::string> services;
    };

    explicit ROS2NodeCache(size_t max_nodes = 500)
        : cache_(max_nodes, 50 * 1024 * 1024, std::chrono::seconds(60)) {}

    /**
     * @brief Get cached node info
     */
    std::optional<NodeCacheEntry> get_node(const std::string& node_name) {
        return cache_.get(node_name);
    }

    /**
     * @brief Cache node info
     */
    void cache_node(const NodeCacheEntry& entry) {
        size_t size = sizeof(NodeCacheEntry) + entry.name.size() +
                     entry.namespace_.size();
        for (const auto& sub : entry.subscriptions)
            size += sub.size();
        for (const auto& pub : entry.publications)
            size += pub.size();
        for (const auto& svc : entry.services)
            size += svc.size();
        
        cache_.put(entry.name, entry, size);
    }

    /**
     * @brief Invalidate node cache
     */
    void invalidate_node(const std::string& node_name) {
        cache_.remove(node_name);
    }

    /**
     * @brief Clear all node cache
     */
    void clear_all() {
        cache_.clear();
    }

    /**
     * @brief Cleanup expired entries
     */
    void cleanup() {
        cache_.cleanup_expired();
    }

private:
    AdaptiveCacheManager<std::string, NodeCacheEntry> cache_;
};
