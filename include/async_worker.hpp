/**
 * @file async_worker.hpp
 * @brief Thread pool for asynchronous task execution
 * @author Dashboard Team
 */

#pragma once

#include <functional>
#include <future>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <memory>

namespace ros2_dashboard {

/**
 * @class AsyncWorker
 * @brief Thread pool for background task execution
 * 
 * Provides thread-safe task queuing and execution with support for
 * futures and callbacks. Configurable thread pool size.
 */
class AsyncWorker {
public:
    /**
     * @brief Constructor with configurable thread count
     * @param num_threads Number of worker threads (1-4, default 2)
     */
    explicit AsyncWorker(int num_threads = 2);

    /**
     * @brief Destructor - waits for pending tasks
     */
    ~AsyncWorker();

    /**
     * @brief Enqueue a task with future result
     * @param task Function to execute
     * @return std::future for result
     */
    template<typename F>
    auto enqueue(F&& task) -> std::future<typename std::result_of<F()>::type> {
        using return_type = typename std::result_of<F()>::type;

        auto packaged_task = std::make_shared<std::packaged_task<return_type()>>(
            std::forward<F>(task));
        
        std::future<return_type> res = packaged_task->get_future();
        
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            if (stop_) throw std::runtime_error("enqueue on stopped thread pool");
            
            tasks_.emplace([packaged_task]() { (*packaged_task)(); });
        }
        
        condition_.notify_one();
        return res;
    }

    /**
     * @brief Enqueue a task with completion callback
     * @param task Function to execute
     * @param callback Function called on completion
     */
    template<typename F, typename C>
    void enqueue_with_callback(F&& task, C&& callback) {
        enqueue([task = std::forward<F>(task), 
                callback = std::forward<C>(callback)]() {
            auto result = task();
            callback(result);
        });
    }

    /**
     * @brief Get number of pending tasks
     * @return Task queue size
     */
    size_t get_pending_tasks() const;

    /**
     * @brief Get number of active worker threads
     * @return Thread count
     */
    int get_thread_count() const;

    /**
     * @brief Wait for all pending tasks to complete
     */
    void wait_for_completion();

    /**
     * @brief Shutdown worker threads
     */
    void shutdown();

private:
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    mutable std::mutex queue_mutex_;
    std::condition_variable condition_;
    bool stop_ = false;

    void worker_thread_();
};

}  // namespace ros2_dashboard
