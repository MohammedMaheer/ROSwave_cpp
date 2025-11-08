/**
 * @file containers/thread_safe_queue.hpp
 * @brief Simple production-grade thread-safe queue (FIFO) with blocking pop.
 */
#pragma once

#include <mutex>
#include <condition_variable>
#include <queue>
#include <optional>

namespace ros2_dashboard::containers {

template<typename T>
class ThreadSafeQueue {
public:
    ThreadSafeQueue() = default;
    ~ThreadSafeQueue() = default;

    // non-copyable
    ThreadSafeQueue(const ThreadSafeQueue&) = delete;
    ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

    // push an item (thread-safe)
    void push(T item) {
        {
            std::lock_guard<std::mutex> lg(mu_);
            queue_.push(std::move(item));
        }
        cv_.notify_one();
    }

    // blocking pop. Returns an item. Blocks until available or shutdown.
    std::optional<T> pop() {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait(lk, [this]() { return !queue_.empty() || shutdown_; });
        if (queue_.empty()) return std::nullopt;
        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    // try pop without blocking
    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lg(mu_);
        if (queue_.empty()) return std::nullopt;
        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lg(mu_);
        return queue_.size();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lg(mu_);
        return queue_.empty();
    }

    void shutdown() {
        {
            std::lock_guard<std::mutex> lg(mu_);
            shutdown_ = true;
        }
        cv_.notify_all();
    }

private:
    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::queue<T> queue_;
    bool shutdown_ = false;
};

} // namespace ros2_dashboard::containers
