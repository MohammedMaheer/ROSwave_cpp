/**
 * @file ml_exporter.hpp
 * @brief Machine Learning dataset export and packaging
 * @author Dashboard Team
 */

#pragma once

#include <string>
#include <vector>
#include <map>
#include <cstdint>
#include <functional>
#include <optional>

namespace ros2_dashboard {

/**
 * @struct ExportConfig
 * @brief Configuration for exporting datasets
 */
struct ExportConfig {
    std::string output_directory;
    std::vector<std::string> bag_files;      ///< Paths to bag files to export
    std::vector<std::string> selected_topics; ///< Topics to include (empty = all)
    std::string metadata_annotation;          ///< Custom metadata notes
    bool compress = true;                     ///< Create .tar.gz archive
    std::string compression_format = "gzip"; ///< gzip or zstd
};

/**
 * @struct ExportMetadata
 * @brief Metadata for exported dataset
 */
struct ExportMetadata {
    std::string export_date;
    std::string export_version = "1.0";
    std::vector<std::string> bag_files;
    std::vector<std::string> topics;
    std::map<std::string, int64_t> message_counts;
    std::string total_duration;
    int64_t total_size_bytes = 0;
    std::string annotation;
};

/**
 * @class MLExporter
 * @brief Exports recorded bags as ML training datasets
 * 
 * Generates standardized export format with metadata:
 * - Raw rosbag2 files
 * - metadata.json (recording info)
 * - schema.json (message definitions)
 * - .tar.gz compressed archive
 */
class MLExporter {
public:
    MLExporter();
    ~MLExporter();

    /**
     * @brief Export single bag with metadata
     * @param bag_path Path to input bag file
     * @param output_path Output directory for export
     * @param selected_topics Topics to export (empty = all)
     * @param metadata Optional metadata annotation
     * @return true if export succeeded
     */
    bool export_bag(const std::string& bag_path,
                   const std::string& output_path,
                   const std::vector<std::string>& selected_topics = {},
                   const std::string& metadata = "");

    /**
     * @brief Export multiple bags in batch operation
     * @param config Export configuration
     * @param progress_callback Optional progress callback
     * @return true if batch export succeeded
     */
    bool export_batch(const ExportConfig& config,
                     std::function<void(int, const std::string&)> progress_callback = nullptr);

    /**
     * @brief Generate metadata.json for bags
     * @param bag_files Vector of bag file paths
     * @param selected_topics Topics to include (empty = all)
     * @return JSON string with metadata
     */
    std::string generate_metadata_json(
        const std::vector<std::string>& bag_files,
        const std::vector<std::string>& selected_topics = {});

    /**
     * @brief Generate schema.json for message types
     * @param bag_files Vector of bag file paths
     * @return JSON string with message schemas
     */
    std::string generate_schema_json(
        const std::vector<std::string>& bag_files);

    /**
     * @brief Create compressed archive of export
     * @param export_dir Export directory to compress
     * @param output_file Output archive path
     * @param compression_format "gzip" or "zstd"
     * @return true if compression succeeded
     */
    bool create_archive(const std::string& export_dir,
                       const std::string& output_file,
                       const std::string& compression_format = "gzip");

    /**
     * @brief Get export operation status
     * @return Map with "status", "progress_percent", "current_file"
     */
    std::map<std::string, std::string> get_export_status();

    /**
     * @brief Cancel ongoing export operation
     */
    void cancel_export();

private:
    bool export_in_progress_ = false;
    int export_progress_percent_ = 0;
    std::string current_export_file_;

    // Internal methods
    bool extract_bag_info_(const std::string& bag_path,
                          ExportMetadata& metadata);
    bool copy_bag_files_(const ExportConfig& config);
    bool write_metadata_file_(const ExportMetadata& metadata,
                             const std::string& output_dir);
    bool write_schema_file_(const ExportConfig& config,
                          const std::string& output_dir);
};

}  // namespace ros2_dashboard
