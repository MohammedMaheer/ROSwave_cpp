#pragma once

#include <chrono>
#include <list>
#include <mutex>
#include <optional>
#include <unordered_map>

namespace ros2_dashboard::containers {

template<typename Key, typename Value>
class CacheWithTimeout {
public:
    using clock = std::chrono::steady_clock;
    using duration = std::chrono::milliseconds;

    explicit CacheWithTimeout(size_t capacity = 128, duration ttl = duration(60000))
        : capacity_(capacity), ttl_(ttl) {}

    void put(const Key& k, const Value& v) {
        std::lock_guard<std::mutex> lg(mu_);
        auto it = map_.find(k);
        auto now = clock::now();
        if (it != map_.end()) {
            order_.erase(it->second.it);
            order_.push_front(k);
            it->second.it = order_.begin();
            it->second.value = v;
            it->second.expiry = now + ttl_;
            return;
        }
        if (map_.size() >= capacity_) {
            Key old = order_.back();
            order_.pop_back();
            map_.erase(old);
        }
        order_.push_front(k);
        map_.emplace(k, Entry{v, order_.begin(), now + ttl_});
    }

    std::optional<Value> get(const Key& k) {
        std::lock_guard<std::mutex> lg(mu_);
        auto it = map_.find(k);
        if (it == map_.end()) return std::nullopt;
        auto now = clock::now();
        if (it->second.expiry < now) {
            order_.erase(it->second.it);
            map_.erase(it);
            return std::nullopt;
        }
        order_.erase(it->second.it);
        order_.push_front(k);
        it->second.it = order_.begin();
        return it->second.value;
    }

    void erase(const Key& k) {
        std::lock_guard<std::mutex> lg(mu_);
        auto it = map_.find(k);
        if (it == map_.end()) return;
        order_.erase(it->second.it);
        map_.erase(it);
    }

    void clear() {
        std::lock_guard<std::mutex> lg(mu_);
        map_.clear();
        order_.clear();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lg(mu_);
        return map_.size();
    }

private:
    struct Entry {
        Value value;
        typename std::list<Key>::iterator it;
        clock::time_point expiry;
    };

    size_t capacity_;
    duration ttl_;
    mutable std::mutex mu_;
    std::list<Key> order_;
    std::unordered_map<Key, Entry> map_;
};

} // namespace ros2_dashboard::containers
