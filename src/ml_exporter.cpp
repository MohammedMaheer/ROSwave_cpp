/**
 * @file ml_exporter.cpp
 * @brief Machine Learning dataset export implementation
 */

#include "ml_exporter.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp>
#include <archive.h>
#include <archive_entry.h>
#include <filesystem>

namespace ros2_dashboard {

using json = nlohmann::json;
namespace fs = std::filesystem;

MLExporter::MLExporter() = default;

MLExporter::~MLExporter() = default;

bool MLExporter::export_bag(const std::string& bag_path,
                           const std::string& output_path,
                           const std::vector<std::string>& selected_topics,
                           const std::string& metadata) {
    try {
        export_in_progress_ = true;
        export_progress_percent_ = 0;
        current_export_file_ = bag_path;

        // Create export directory
        fs::create_directories(output_path);

        // Copy bag file
        ExportConfig config;
        config.output_directory = output_path;
        config.bag_files.push_back(bag_path);
        config.selected_topics = selected_topics;
        config.metadata_annotation = metadata;

        export_progress_percent_ = 33;

        // Generate metadata
        auto metadata_json = generate_metadata_json({bag_path}, selected_topics);
        std::ofstream metadata_file(output_path + "/metadata.json");
        metadata_file << metadata_json;
        metadata_file.close();

        export_progress_percent_ = 66;

        // Generate schema
        auto schema_json = generate_schema_json({bag_path});
        std::ofstream schema_file(output_path + "/schema.json");
        schema_file << schema_json;
        schema_file.close();

        export_progress_percent_ = 100;
        export_in_progress_ = false;

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error exporting bag: " << e.what() << std::endl;
        export_in_progress_ = false;
        return false;
    }
}

bool MLExporter::export_batch(const ExportConfig& config,
                             std::function<void(int, const std::string&)> progress_callback) {
    try {
        export_in_progress_ = true;
        export_progress_percent_ = 0;

        fs::create_directories(config.output_directory);

        int total_files = config.bag_files.size();
        int processed = 0;

        for (const auto& bag : config.bag_files) {
            current_export_file_ = bag;
            
            if (progress_callback) {
                progress_callback(processed * 100 / total_files, bag);
            }

            // Copy bag file to output directory
            std::string dest = config.output_directory + "/" + 
                             fs::path(bag).filename().string();
            fs::copy_file(bag, dest, fs::copy_options::overwrite_existing);

            processed++;
            export_progress_percent_ = (processed * 90) / total_files;
        }

        // Generate combined metadata
        auto metadata_json = generate_metadata_json(
            config.bag_files, config.selected_topics);
        std::ofstream metadata_file(
            config.output_directory + "/metadata.json");
        metadata_file << metadata_json;
        metadata_file.close();

        export_progress_percent_ = 95;

        // Create archive if requested
        if (config.compress) {
            std::string archive_path = config.output_directory + ".tar.gz";
            if (create_archive(config.output_directory, archive_path,
                             config.compression_format)) {
                export_progress_percent_ = 100;
            }
        }

        export_in_progress_ = false;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error in batch export: " << e.what() << std::endl;
        export_in_progress_ = false;
        return false;
    }
}

std::string MLExporter::generate_metadata_json(
    const std::vector<std::string>& bag_files,
    const std::vector<std::string>& selected_topics) {
    try {
        json metadata;
        metadata["export_version"] = "1.0";
        metadata["export_date"] = "2025-01-01T00:00:00Z";  // TODO: Get current time
        metadata["bag_files"] = bag_files;
        metadata["topics"] = selected_topics;
        metadata["message_counts"] = json::object();
        metadata["total_duration"] = "0:00:00";  // TODO: Calculate
        metadata["total_size_bytes"] = 0;  // TODO: Calculate
        
        return metadata.dump(2);
    } catch (const std::exception& e) {
        std::cerr << "Error generating metadata: " << e.what() << std::endl;
        return "{}";
    }
}

std::string MLExporter::generate_schema_json(
    const std::vector<std::string>& bag_files) {
    try {
        json schema;
        schema["topics"] = json::array();
        
        // TODO: Extract message types and generate schemas
        
        return schema.dump(2);
    } catch (const std::exception& e) {
        std::cerr << "Error generating schema: " << e.what() << std::endl;
        return "{}";
    }
}

bool MLExporter::create_archive(const std::string& export_dir,
                               const std::string& output_file,
                               const std::string& compression_format) {
    try {
        struct archive* a = archive_write_new();
        
        // Select compression format
        if (compression_format == "zstd") {
            archive_write_add_filter_zstd(a);
        } else {
            archive_write_add_filter_gzip(a);
        }
        
        archive_write_set_format_pax_restricted(a);
        
        if (archive_write_open_filename(a, output_file.c_str()) != ARCHIVE_OK) {
            archive_write_free(a);
            return false;
        }

        // Add files to archive
        for (const auto& entry : fs::recursive_directory_iterator(export_dir)) {
            if (entry.is_regular_file()) {
                struct archive_entry* ae = archive_entry_new();
                
                std::string pathname = entry.path().string();
                archive_entry_set_pathname(ae, pathname.c_str());
                archive_entry_set_size(ae, fs::file_size(entry));
                archive_entry_set_filetype(ae, AE_IFREG);
                archive_entry_set_perm(ae, 0644);

                archive_write_header(a, ae);

                std::ifstream file(pathname, std::ios::binary);
                char buf[4096];
                while (file.read(buf, sizeof(buf))) {
                    archive_write_data(a, buf, file.gcount());
                }
                file.close();

                archive_entry_free(ae);
            }
        }

        archive_write_close(a);
        archive_write_free(a);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error creating archive: " << e.what() << std::endl;
        return false;
    }
}

std::map<std::string, std::string> MLExporter::get_export_status() {
    std::map<std::string, std::string> status;
    status["status"] = export_in_progress_ ? "in_progress" : "idle";
    status["progress_percent"] = std::to_string(export_progress_percent_);
    status["current_file"] = current_export_file_;
    return status;
}

void MLExporter::cancel_export() {
    export_in_progress_ = false;
}

bool MLExporter::extract_bag_info_(const std::string& bag_path,
                                  ExportMetadata& metadata) {
    // TODO: Use rosbag2 API to extract metadata
    return true;
}

bool MLExporter::copy_bag_files_(const ExportConfig& config) {
    try {
        for (const auto& bag : config.bag_files) {
            std::string dest = config.output_directory + "/" + 
                             fs::path(bag).filename().string();
            fs::copy_file(bag, dest, fs::copy_options::overwrite_existing);
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error copying bag files: " << e.what() << std::endl;
        return false;
    }
}

bool MLExporter::write_metadata_file_(const ExportMetadata& metadata,
                                     const std::string& output_dir) {
    try {
        json j;
        j["export_date"] = metadata.export_date;
        j["export_version"] = metadata.export_version;
        j["bag_files"] = metadata.bag_files;
        j["topics"] = metadata.topics;
        j["message_counts"] = metadata.message_counts;
        j["total_duration"] = metadata.total_duration;
        j["total_size_bytes"] = metadata.total_size_bytes;
        j["annotation"] = metadata.annotation;

        std::string filepath = output_dir + "/metadata.json";
        std::ofstream file(filepath);
        file << j.dump(2);
        file.close();
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error writing metadata: " << e.what() << std::endl;
        return false;
    }
}

bool MLExporter::write_schema_file_(const ExportConfig& config,
                                   const std::string& output_dir) {
    // TODO: Extract and write message schemas
    return true;
}

}  // namespace ros2_dashboard
