/**
 * @file network_manager.hpp
 * @brief Offline-first cloud integration with queued uploads
 * @author Dashboard Team
 */

#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <cstdint>
#include <optional>

namespace ros2_dashboard {

/**
 * @enum UploadStatus
 * @brief Current status of an upload
 */
enum class UploadStatus {
    PENDING,      ///< Waiting to upload
    UPLOADING,    ///< Currently uploading
    COMPLETED,    ///< Successfully completed
    FAILED,       ///< Upload failed
    CANCELLED     ///< User cancelled
};

/**
 * @struct UploadTask
 * @brief A single upload task in the queue
 */
struct UploadTask {
    std::string id;                      ///< Unique task identifier
    std::string file_path;               ///< Local file path
    UploadStatus status = UploadStatus::PENDING;
    int priority = 5;                    ///< 1-10, 10 is highest
    int64_t file_size_bytes = 0;
    int64_t uploaded_bytes = 0;
    int retry_count = 0;
    int max_retries = 3;
    std::string error_message;
    uint64_t created_timestamp_ms = 0;
    uint64_t completed_timestamp_ms = 0;
};

/**
 * @class NetworkManager
 * @brief Manages offline-first uploads with persistent queue
 * 
 * Features:
 * - SQLite-backed persistent queue
 * - Chunked uploads with resume capability
 * - Exponential backoff retry logic
 * - Priority-based queue management
 * - Bandwidth throttling support
 * - Upload callbacks for monitoring
 */
class NetworkManager {
public:
    NetworkManager();
    ~NetworkManager();

    /**
     * @brief Initialize network manager with configuration
     * @param db_path Path to SQLite database for queue storage
     * @param config Configuration JSON with upload settings
     * @return true if initialization succeeded
     */
    bool initialize(const std::string& db_path,
                    const std::string& config_path = "");

    /**
     * @brief Add a file to the upload queue
     * @param file_path Path to file to upload
     * @param priority Priority level (1-10, default 5)
     * @param destination Cloud endpoint identifier
     * @return Upload task ID
     */
    std::string queue_upload(const std::string& file_path,
                            int priority = 5,
                            const std::string& destination = "default");

    /**
     * @brief Start processing the upload queue
     * @param max_concurrent Maximum concurrent uploads
     * @return true if queue processing started
     */
    bool start_queue_processing(int max_concurrent = 2);

    /**
     * @brief Stop processing uploads gracefully
     */
    void stop_queue_processing();

    /**
     * @brief Get all queued uploads
     * @return Vector of UploadTask structs
     */
    std::vector<UploadTask> get_queue();

    /**
     * @brief Get upload task status
     * @param task_id Task ID returned from queue_upload
     * @return UploadTask if found
     */
    std::optional<UploadTask> get_task_status(const std::string& task_id);

    /**
     * @brief Retry a failed upload
     * @param task_id Task ID to retry
     * @return true if retry queued
     */
    bool retry_upload(const std::string& task_id);

    /**
     * @brief Cancel an upload
     * @param task_id Task ID to cancel
     * @return true if cancelled
     */
    bool cancel_upload(const std::string& task_id);

    /**
     * @brief Clear completed uploads from history
     * @return Number of cleared tasks
     */
    int clear_completed();

    /**
     * @brief Set chunk size for uploads
     * @param chunk_size_mb Chunk size in MB (default 5)
     */
    void set_chunk_size(int chunk_size_mb);

    /**
     * @brief Set bandwidth throttle limit
     * @param mbps_limit Limit in MB/s (0 = unlimited)
     */
    void set_bandwidth_limit(double mbps_limit);

    /**
     * @brief Set upload endpoint configuration
     * @param endpoint_url HTTP endpoint URL
     * @param api_key API key for authentication
     * @param auth_token Optional bearer token
     */
    void set_endpoint(const std::string& endpoint_url,
                      const std::string& api_key = "",
                      const std::string& auth_token = "");

    /**
     * @brief Callback function type for upload progress
     */
    using UploadProgressCallback = std::function<void(
        const std::string& task_id,
        int64_t uploaded_bytes,
        int64_t total_bytes)>;

    /**
     * @brief Callback function type for upload completion
     */
    using UploadCompleteCallback = std::function<void(
        const std::string& task_id,
        bool success,
        const std::string& error_message)>;

    /**
     * @brief Register progress callback
     * @param callback Function to call on progress
     */
    void set_progress_callback(UploadProgressCallback callback);

    /**
     * @brief Register completion callback
     * @param callback Function to call on completion
     */
    void set_complete_callback(UploadCompleteCallback callback);

    /**
     * @brief Get network connectivity status
     * @return true if connected to network
     */
    bool is_network_available();

    /**
     * @brief Get queue statistics
     * @return Map with keys: "total_tasks", "pending", "uploading", 
     *         "completed", "failed", "total_size_mb"
     */
    std::map<std::string, std::string> get_queue_stats();

private:
    std::string db_path_;
    std::string endpoint_url_;
    std::string api_key_;
    std::string auth_token_;
    int chunk_size_mb_ = 5;
    double bandwidth_limit_mbps_ = 0.0;  // 0 = unlimited
    bool queue_processing_ = false;
    int max_concurrent_uploads_ = 2;

    UploadProgressCallback progress_callback_;
    UploadCompleteCallback complete_callback_;

    // Internal methods
    bool upload_task_(UploadTask& task);
    bool upload_chunk_(const std::string& file_path,
                      int64_t offset,
                      int64_t chunk_size,
                      const std::string& task_id);
    bool load_queue_from_db_();
    bool save_queue_to_db_();
    void process_queue_();
};

}  // namespace ros2_dashboard
