/**
 * @file tests/test_containers.cpp
 * @brief Unit tests for container utilities
 */

#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include "../include/containers/bounded_deque.hpp"
#include "../include/containers/resource_pool.hpp"
#include "../include/containers/cache_with_timeout.hpp"
#include "../include/containers/thread_safe_queue.hpp"
#include "../include/metrics_history_buffer.hpp"

using namespace ros2_dashboard::containers;

// ===== BoundedDeque Tests =====
TEST(BoundedDequeTest, PushAndPop) {
    BoundedDeque<int, 5> deque;
    deque.push_back(1);
    deque.push_back(2);
    deque.push_back(3);
    
    EXPECT_EQ(deque.size(), 3);
    EXPECT_EQ(deque[0], 1);
    EXPECT_EQ(deque[2], 3);
}

TEST(BoundedDequeTest, AutoEviction) {
    BoundedDeque<int, 3> deque;
    deque.push_back(1);
    deque.push_back(2);
    deque.push_back(3);
    deque.push_back(4);  // Should evict 1
    
    EXPECT_EQ(deque.size(), 3);
    EXPECT_EQ(deque[0], 2);
    EXPECT_EQ(deque[2], 4);
}

TEST(BoundedDequeTest, Full) {
    BoundedDeque<int, 2> deque;
    deque.push_back(1);
    EXPECT_FALSE(deque.full());
    deque.push_back(2);
    EXPECT_TRUE(deque.full());
}

TEST(BoundedDequeTest, Clear) {
    BoundedDeque<int, 5> deque;
    deque.push_back(1);
    deque.push_back(2);
    deque.clear();
    EXPECT_EQ(deque.size(), 0);
    EXPECT_TRUE(deque.empty());
}

// ===== MetricsHistoryBuffer Tests =====
TEST(MetricsHistoryBufferTest, PushAndRetrieve) {
    ros2_dashboard::MetricsHistoryBuffer buffer;
    
    ros2_dashboard::MetricsPoint pt;
    pt.timestamp_ms = 1000;
    pt.cpu_percent = 42.5;
    pt.memory_mb = 256;
    
    buffer.push_point(std::move(pt));
    
    EXPECT_EQ(buffer.size(), 1);
    EXPECT_FALSE(buffer.empty());
}

TEST(MetricsHistoryBufferTest, Statistics) {
    ros2_dashboard::MetricsHistoryBuffer buffer;
    
    for (int i = 0; i < 10; ++i) {
        ros2_dashboard::MetricsPoint pt;
        pt.cpu_percent = 10.0 + i * 5.0;  // 10, 15, 20, ..., 55
        buffer.push_point(pt);
    }
    
    auto stats = buffer.get_cpu_stats();
    EXPECT_DOUBLE_EQ(stats.min, 10.0);
    EXPECT_DOUBLE_EQ(stats.max, 55.0);
    EXPECT_DOUBLE_EQ(stats.avg, 32.5);  // (10+15+...+55) / 10
}

TEST(MetricsHistoryBufferTest, BoundedCapacity) {
    ros2_dashboard::MetricsHistoryBuffer buffer;
    
    // Add more than capacity (600 default)
    for (int i = 0; i < 700; ++i) {
        ros2_dashboard::MetricsPoint pt;
        pt.cpu_percent = static_cast<double>(i % 100);
        buffer.push_point(pt);
    }
    
    // Should be bounded to 600
    EXPECT_LE(buffer.size(), 600);
}

// ===== ThreadSafeQueue Tests =====
TEST(ThreadSafeQueueTest, PushAndTryPop) {
    ThreadSafeQueue<int> queue;
    
    queue.push(42);
    auto val = queue.try_pop();
    EXPECT_TRUE(val.has_value());
    EXPECT_EQ(*val, 42);
}

TEST(ThreadSafeQueueTest, EmptyQueueTryPop) {
    ThreadSafeQueue<int> queue;
    auto val = queue.try_pop();
    EXPECT_FALSE(val.has_value());
}

TEST(ThreadSafeQueueTest, MultiThreadedPush) {
    ThreadSafeQueue<int> queue;
    
    std::thread t1([&queue]() {
        for (int i = 0; i < 100; ++i) queue.push(i);
    });
    
    std::thread t2([&queue]() {
        for (int i = 100; i < 200; ++i) queue.push(i);
    });
    
    t1.join();
    t2.join();
    
    int count = 0;
    while (auto val = queue.try_pop()) {
        count++;
    }
    EXPECT_EQ(count, 200);
}

TEST(ThreadSafeQueueTest, WaitAndPop) {
    ThreadSafeQueue<int> queue;
    
    std::thread pusher([&queue]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        queue.push(99);
    });
    
    auto val = queue.pop();
    EXPECT_TRUE(val.has_value());
    EXPECT_EQ(*val, 99);
    
    pusher.join();
}

TEST(ThreadSafeQueueTest, PopAfterShutdown) {
    ThreadSafeQueue<int> queue;
    
    std::thread pusher([&queue]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        queue.shutdown();
    });
    
    auto val = queue.pop();
    // After shutdown with empty queue, should return nullopt
    EXPECT_FALSE(val.has_value());
    
    pusher.join();
}

// ===== CacheWithTimeout Tests =====
TEST(CacheWithTimeoutTest, GetAndSet) {
    CacheWithTimeout<std::string, int> cache;
    
    cache.put("key1", 42);
    auto val = cache.get("key1");
    EXPECT_TRUE(val.has_value());
    EXPECT_EQ(*val, 42);
}

TEST(CacheWithTimeoutTest, Expiration) {
    CacheWithTimeout<std::string, int> cache(128, std::chrono::milliseconds(100));
    
    cache.put("key1", 42);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    
    auto val = cache.get("key1");
    EXPECT_FALSE(val.has_value());
}

TEST(CacheWithTimeoutTest, LRUEviction) {
    CacheWithTimeout<std::string, int> cache(3);  // capacity = 3
    
    cache.put("key1", 1);
    cache.put("key2", 2);
    cache.put("key3", 3);
    cache.put("key4", 4);  // Should evict key1
    
    auto val1 = cache.get("key1");
    EXPECT_FALSE(val1.has_value());
    
    auto val4 = cache.get("key4");
    EXPECT_TRUE(val4.has_value());
    EXPECT_EQ(*val4, 4);
}

// ===== ResourcePool Tests =====
TEST(ResourcePoolTest, AcquireAndRelease) {
    struct TestObject {
        int value = 42;
    };
    
    ResourcePool<TestObject> pool([]() { return new TestObject{}; });
    
    {
        auto obj1 = pool.acquire();
        EXPECT_EQ(obj1->value, 42);
    }
    
    // After releasing, another acquire should get the same object back
    auto obj2 = pool.acquire();
    EXPECT_EQ(obj2->value, 42);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
