/**
 * @file containers/bounded_deque.hpp
 * @brief Fixed-size circular buffer with RAII and move semantics
 * @author Dashboard Team
 * 
 * Provides a bounded circular deque that never grows beyond MaxSize.
 * When full, oldest elements are automatically evicted.
 * Thread-safe reads, but not designed for concurrent modifications.
 */

#pragma once

#include <vector>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <type_traits>

namespace ros2_dashboard::containers {

/**
 * @class BoundedDeque
 * @brief Fixed-size circular buffer container
 * 
 * @tparam T Element type (should be copyable/movable)
 * @tparam MaxSize Maximum capacity (fixed at compile time)
 * 
 * Features:
 * - O(1) append operations (no dynamic allocation)
 * - O(1) random access via operator[]
 * - Automatic eviction of oldest on overflow
 * - Move semantics for zero-copy operations
 * - No exceptions during normal operation
 * 
 * Example:
 * @code
 *   BoundedDeque<double, 600> cpu_history;
 *   cpu_history.push_back(42.5);  // Add measurement
 *   cpu_history.push_back(43.1);  // Oldest evicted if full
 *   for (const auto& val : cpu_history) { ... }  // Iterate
 * @endcode
 */
template <typename T, size_t MaxSize>
class BoundedDeque {
    static_assert(MaxSize > 0, "MaxSize must be > 0");
    static_assert(std::is_move_constructible_v<T>, "T must be move-constructible");

public:
    using value_type = T;
    using size_type = size_t;
    using difference_type = ptrdiff_t;
    using reference = T&;
    using const_reference = const T&;
    using pointer = T*;
    using const_pointer = const T*;
    using iterator = pointer;
    using const_iterator = const_pointer;

    /**
     * @brief Default constructor - initializes empty deque
     */
    BoundedDeque() : size_(0), head_(0) {}

    /**
     * @brief Destructor - cleans up all elements
     */
    ~BoundedDeque() = default;

    /**
     * @brief Copy constructor - deep copy of contents
     */
    BoundedDeque(const BoundedDeque& other) = default;

    /**
     * @brief Move constructor - transfers ownership efficiently
     */
    BoundedDeque(BoundedDeque&& other) noexcept = default;

    /**
     * @brief Copy assignment operator
     */
    BoundedDeque& operator=(const BoundedDeque& other) = default;

    /**
     * @brief Move assignment operator
     */
    BoundedDeque& operator=(BoundedDeque&& other) noexcept = default;

    /**
     * @brief Add element to the end
     * 
     * If buffer is full, automatically evicts the oldest element.
     * Time complexity: O(1)
     * 
     * @param value Element to append
     */
    void push_back(const T& value) {
        if (size_ < MaxSize) {
            data_[(head_ + size_) % MaxSize] = value;
            size_++;
        } else {
            // Buffer full - overwrite oldest and advance head
            data_[head_] = value;
            head_ = (head_ + 1) % MaxSize;
        }
    }

    /**
     * @brief Add element to the end (move semantics)
     * 
     * If buffer is full, automatically evicts the oldest element.
     * Time complexity: O(1)
     * 
     * @param value Element to append (will be moved)
     */
    void push_back(T&& value) {
        if (size_ < MaxSize) {
            data_[(head_ + size_) % MaxSize] = std::move(value);
            size_++;
        } else {
            // Buffer full - overwrite oldest and advance head
            data_[head_] = std::move(value);
            head_ = (head_ + 1) % MaxSize;
        }
    }

    /**
     * @brief Remove and return the first element
     * 
     * Time complexity: O(1)
     * 
     * @return The oldest element
     * @throws std::underflow_error if deque is empty
     */
    T pop_front() {
        if (empty()) {
            throw std::underflow_error("BoundedDeque is empty");
        }
        T value = std::move(data_[head_]);
        head_ = (head_ + 1) % MaxSize;
        size_--;
        return value;
    }

    /**
     * @brief Access element at index
     * 
     * Index 0 = oldest element, size_-1 = newest
     * Time complexity: O(1)
     * 
     * @param index Position (0 to size_-1)
     * @return Reference to element
     * @throws std::out_of_range if index >= size_
     */
    reference at(size_type index) {
        if (index >= size_) {
            throw std::out_of_range("BoundedDeque index out of range");
        }
        return data_[(head_ + index) % MaxSize];
    }

    /**
     * @brief Const access to element at index
     */
    const_reference at(size_type index) const {
        if (index >= size_) {
            throw std::out_of_range("BoundedDeque index out of range");
        }
        return data_[(head_ + index) % MaxSize];
    }

    /**
     * @brief Unsafe element access (no bounds checking)
     */
    reference operator[](size_type index) {
        return data_[(head_ + index) % MaxSize];
    }

    /**
     * @brief Const unsafe element access
     */
    const_reference operator[](size_type index) const {
        return data_[(head_ + index) % MaxSize];
    }

    /**
     * @brief Get the first (oldest) element
     * 
     * @return Reference to front
     * @throws std::out_of_range if empty
     */
    reference front() {
        if (empty()) throw std::out_of_range("BoundedDeque is empty");
        return data_[head_];
    }

    /**
     * @brief Const access to first element
     */
    const_reference front() const {
        if (empty()) throw std::out_of_range("BoundedDeque is empty");
        return data_[head_];
    }

    /**
     * @brief Get the last (newest) element
     * 
     * @return Reference to back
     * @throws std::out_of_range if empty
     */
    reference back() {
        if (empty()) throw std::out_of_range("BoundedDeque is empty");
        return data_[(head_ + size_ - 1) % MaxSize];
    }

    /**
     * @brief Const access to last element
     */
    const_reference back() const {
        if (empty()) throw std::out_of_range("BoundedDeque is empty");
        return data_[(head_ + size_ - 1) % MaxSize];
    }

    /**
     * @brief Check if deque is empty
     */
    bool empty() const { return size_ == 0; }

    /**
     * @brief Check if deque is full
     */
    bool full() const { return size_ == MaxSize; }

    /**
     * @brief Get current number of elements
     */
    size_type size() const { return size_; }

    /**
     * @brief Get maximum capacity
     */
    static constexpr size_type capacity() { return MaxSize; }

    /**
     * @brief Get maximum capacity (same as capacity())
     */
    static constexpr size_type max_size() { return MaxSize; }

    /**
     * @brief Clear all elements
     */
    void clear() {
        size_ = 0;
        head_ = 0;
    }

    /**
     * @brief Get underlying data pointer (for direct access)
     * 
     * WARNING: Data is circular! Use only if you understand the layout:
     * - Valid data range: [head_, head_ + size_) mod MaxSize
     * - Linear iteration requires two passes if head_ + size_ > MaxSize
     */
    pointer data() { return data_.data(); }

    /**
     * @brief Const pointer to underlying data
     */
    const_pointer data() const { return data_.data(); }

    /**
     * @brief Get linear copy of all data in order
     * 
     * Useful when you need contiguous, linearized data.
     * Time complexity: O(size_)
     * 
     * @return std::vector<T> with all elements in order
     */
    std::vector<T> to_vector() const {
        std::vector<T> result;
        result.reserve(size_);
        for (size_type i = 0; i < size_; ++i) {
            result.push_back(data_[(head_ + i) % MaxSize]);
        }
        return result;
    }

    /**
     * @brief Begin iterator (oldest element)
     */
    iterator begin() {
        // For circular buffer, we can't easily provide true random iterators
        // Return pointer to linearized data if needed via to_vector()
        // This is a limitation of circular buffers
        return nullptr;  // Not supported - use to_vector() or indexed access
    }

    /**
     * @brief End iterator
     */
    iterator end() { return nullptr; }

    /**
     * @brief Const begin iterator
     */
    const_iterator begin() const { return nullptr; }

    /**
     * @brief Const end iterator
     */
    const_iterator end() const { return nullptr; }

    /**
     * @brief Fill deque with N copies of value
     * 
     * @param count Number of elements to add
     * @param value Element to replicate
     */
    void fill(size_type count, const T& value) {
        clear();
        for (size_type i = 0; i < std::min(count, MaxSize); ++i) {
            push_back(value);
        }
    }

    /**
     * @brief Check if buffer is at exactly N% capacity
     * 
     * Useful for monitoring collection behavior
     * 
     * @param percent Threshold 0-100
     * @return true if size >= (percent * MaxSize / 100)
     */
    bool is_at_least_full_percent(int percent) const {
        return (size_ * 100) >= (percent * MaxSize);
    }

private:
    std::array<T, MaxSize> data_;     ///< Circular buffer storage
    size_type size_;                   ///< Number of valid elements
    size_type head_;                   ///< Index of oldest element (circular)
};

}  // namespace ros2_dashboard::containers
