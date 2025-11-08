#pragma once

#include <vector>
#include <memory>
#include <mutex>
#include <functional>

namespace ros2_dashboard::containers {

template <typename T>
class ResourcePool {
public:
    using unique_resource_t = std::unique_ptr<T, std::function<void(T*)>>;

    ResourcePool(std::function<T*()> factory, size_t initial = 0)
        : factory_(std::move(factory)), shutdown_(false) {
        for (size_t i = 0; i < initial; ++i) {
            T* raw = factory_();
            free_.push_back(std::unique_ptr<T>(raw));
        }
    }

    ~ResourcePool() {
        std::lock_guard<std::mutex> lg(mu_);
        shutdown_ = true;
        free_.clear();
    }

    unique_resource_t acquire() {
        std::unique_lock<std::mutex> lk(mu_);
        if (shutdown_) return unique_resource_t(nullptr, [](T*){});
        
        if (free_.empty()) {
            T* raw = factory_();
            auto deleter = [this](T* p) {
                std::lock_guard<std::mutex> lg(mu_);
                free_.emplace_back(std::unique_ptr<T>(p));
            };
            return unique_resource_t(raw, deleter);
        }
        
        auto up = std::move(free_.back());
        free_.pop_back();
        T* raw = up.release();

        auto deleter = [this](T* p) {
            std::lock_guard<std::mutex> lg(mu_);
            free_.emplace_back(std::unique_ptr<T>(p));
        };

        return unique_resource_t(raw, deleter);
    }

    void add(std::unique_ptr<T> resource) {
        std::lock_guard<std::mutex> lg(mu_);
        free_.push_back(std::move(resource));
    }

private:
    ResourcePool(const ResourcePool&) = delete;
    ResourcePool& operator=(const ResourcePool&) = delete;

    std::function<T*()> factory_;
    std::vector<std::unique_ptr<T>> free_;
    std::mutex mu_;
    bool shutdown_;
};

} // namespace ros2_dashboard::containers
