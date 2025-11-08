/**
 * @file async_worker.cpp
 * @brief Thread pool implementation
 */

#include "async_worker.hpp"
#include <iostream>

namespace ros2_dashboard {

AsyncWorker::AsyncWorker(int num_threads)
    : stop_(false) {
    int thread_count = std::max(1, std::min(4, num_threads));
    
    for (int i = 0; i < thread_count; ++i) {
        workers_.emplace_back([this]() { worker_thread_(); });
    }
}

AsyncWorker::~AsyncWorker() {
    shutdown();
}

size_t AsyncWorker::get_pending_tasks() const {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    return tasks_.size();
}

int AsyncWorker::get_thread_count() const {
    return workers_.size();
}

void AsyncWorker::wait_for_completion() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    condition_.wait(lock, [this]() { return tasks_.empty(); });
}

void AsyncWorker::shutdown() {
    {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        stop_ = true;
    }
    condition_.notify_all();

    for (auto& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

void AsyncWorker::worker_thread_() {
    while (true) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        condition_.wait(lock, [this]() { 
            return !tasks_.empty() || stop_; 
        });

        if (stop_ && tasks_.empty()) {
            break;
        }

        if (!tasks_.empty()) {
            auto task = std::move(tasks_.front());
            tasks_.pop();
            lock.unlock();

            try {
                task();
            } catch (const std::exception& e) {
                std::cerr << "Task execution error: " << e.what() << std::endl;
            }
        }
    }
}

}  // namespace ros2_dashboard
