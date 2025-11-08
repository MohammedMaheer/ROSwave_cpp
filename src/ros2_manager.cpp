/**
 * @file ros2_manager.cpp
 * @brief ROS2 system discovery and management implementation
 */

#include "ros2_manager.hpp"
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <memory>

namespace ros2_dashboard {

ROS2Manager::ROS2Manager()
    : cache_ttl_seconds_(5), cache_hits_(0), cache_misses_(0), 
      recording_active_(false) {
}

ROS2Manager::~ROS2Manager() {
    if (recording_active_) {
        stop_recording();
    }
}

std::vector<TopicInfo> ROS2Manager::get_topics() {
    std::lock_guard<std::mutex> lock(cache_mutex_);

    // Check cache validity
    if (topics_cache_.has_value() && 
        topics_cache_->is_valid(cache_ttl_seconds_)) {
        cache_hits_++;
        return topics_cache_->data;
    }

    cache_misses_++;
    auto topics = discover_topics_();
    
    topics_cache_ = CacheEntry<std::vector<TopicInfo>>{
        topics,
        std::chrono::steady_clock::now()
    };

    return topics;
}

std::vector<NodeInfo> ROS2Manager::get_nodes() {
    std::lock_guard<std::mutex> lock(cache_mutex_);

    if (nodes_cache_.has_value() && 
        nodes_cache_->is_valid(cache_ttl_seconds_)) {
        cache_hits_++;
        return nodes_cache_->data;
    }

    cache_misses_++;
    auto nodes = discover_nodes_();
    
    nodes_cache_ = CacheEntry<std::vector<NodeInfo>>{
        nodes,
        std::chrono::steady_clock::now()
    };

    return nodes;
}

std::vector<ServiceInfo> ROS2Manager::get_services() {
    std::lock_guard<std::mutex> lock(cache_mutex_);

    if (services_cache_.has_value() && 
        services_cache_->is_valid(cache_ttl_seconds_)) {
        cache_hits_++;
        return services_cache_->data;
    }

    cache_misses_++;
    auto services = discover_services_();
    
    services_cache_ = CacheEntry<std::vector<ServiceInfo>>{
        services,
        std::chrono::steady_clock::now()
    };

    return services;
}

std::optional<std::string> ROS2Manager::get_topic_echo(
    const std::string& topic_name) {
    try {
        // Use ros2 topic echo command with timeout (removed --limit which is not valid)
        std::string cmd = "timeout 1 ros2 topic echo " + topic_name + 
                         " --no-arr 2>/dev/null | head -30";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            std::cerr << "[ROS2Manager] Failed to execute topic echo command" << std::endl;
            return std::nullopt;
        }

        std::string result;
        char buffer[256];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
        int status = pclose(pipe);
        
        if (status != 0) {
            std::cerr << "[ROS2Manager] Topic echo command failed with status: " << status << std::endl;
            return std::nullopt;
        }

        return result.empty() ? std::nullopt : std::optional<std::string>(result);
    } catch (const std::exception& e) {
        std::cerr << "Error echoing topic: " << e.what() << std::endl;
        return std::nullopt;
    }
}

bool ROS2Manager::start_recording(const std::string& output_dir,
                                 const std::vector<std::string>& topic_filter,
                                 const std::string& compression_format) {
    if (recording_active_) {
        std::cerr << "Recording already active" << std::endl;
        return false;
    }

    try {
        std::string cmd = "ros2 bag record -o " + output_dir;
        cmd += " --compression-format " + compression_format;

        if (!topic_filter.empty()) {
            for (const auto& topic : topic_filter) {
                cmd += " " + topic;
            }
        }

        // Start recording in background
        cmd += " &";
        system(cmd.c_str());

        recording_active_ = true;
        recording_start_time_ = std::chrono::steady_clock::now();
        recording_output_dir_ = output_dir;
        std::cerr << "[ROS2Manager] Recording started to " << output_dir << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error starting recording: " << e.what() << std::endl;
        return false;
    }
}

bool ROS2Manager::stop_recording() {
    if (!recording_active_) {
        return false;
    }

    try {
        system("pkill -f 'ros2 bag record'");
        recording_active_ = false;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error stopping recording: " << e.what() << std::endl;
        return false;
    }
}

bool ROS2Manager::is_recording() const {
    return recording_active_;
}

std::map<std::string, std::string> ROS2Manager::get_recording_status() {
    std::map<std::string, std::string> status;
    status["active"] = recording_active_ ? "true" : "false";
    
    if (recording_active_) {
        // Calculate elapsed time
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - recording_start_time_).count();
        
        int hours = elapsed / 3600;
        int minutes = (elapsed % 3600) / 60;
        int seconds = elapsed % 60;
        
        char time_buf[32];
        snprintf(time_buf, sizeof(time_buf), "%02d:%02d:%02d", hours, minutes, seconds);
        status["elapsed_time"] = time_buf;
        
        // Calculate file size from directory
        std::string cmd = "du -sh '" + recording_output_dir_ + "' 2>/dev/null | awk '{print $1}'";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (pipe) {
            char size_buf[64];
            if (fgets(size_buf, sizeof(size_buf), pipe) != nullptr) {
                std::string size_str(size_buf);
                // Remove newline and carriage returns
                size_str.erase(std::remove(size_str.begin(), size_str.end(), '\n'), size_str.end());
                size_str.erase(std::remove(size_str.begin(), size_str.end(), '\r'), size_str.end());
                // Remove leading/trailing whitespace
                size_t start = size_str.find_first_not_of(" \t");
                if (start != std::string::npos) {
                    size_str = size_str.substr(start);
                    size_t end = size_str.find_last_not_of(" \t");
                    if (end != std::string::npos) {
                        size_str = size_str.substr(0, end + 1);
                    }
                }
                status["file_size"] = size_str.empty() ? "0 B" : size_str;
            } else {
                status["file_size"] = "0 B";
            }
            int close_status = pclose(pipe);
            if (close_status != 0) {
                std::cerr << "[ROS2Manager] du command failed, status: " << close_status << std::endl;
            }
        } else {
            status["file_size"] = "0 B";
            std::cerr << "[ROS2Manager] Failed to execute du command" << std::endl;
        }
    } else {
        status["elapsed_time"] = "00:00:00";
        status["file_size"] = "0 B";
    }
    
    return status;
}

ROS2Manager::BagMetadata ROS2Manager::extract_bag_metadata(
    const std::string& bag_path) {
    BagMetadata metadata;
    metadata.filename = bag_path;
    metadata.created_time = "2025-01-01T00:00:00Z";  // TODO: Implement proper time extraction

    try {
        std::string cmd = "ros2 bag info " + bag_path + " --verbose";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            std::cerr << "[ROS2Manager] Failed to execute bag info command: " << bag_path << std::endl;
            return metadata;
        }

        char buffer[256];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            // Parse bag info output
            std::string line(buffer);
            // TODO: Parse topics, message counts, duration from output
        }
        int status = pclose(pipe);
        if (status != 0) {
            std::cerr << "[ROS2Manager] Bag info command failed with status: " << status << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error extracting bag metadata: " << e.what() << std::endl;
    }

    return metadata;
}

std::vector<ROS2Manager::BagMetadata> ROS2Manager::list_bags(
    const std::string& directory) {
    std::vector<BagMetadata> bags;
    // TODO: Implement bag listing
    return bags;
}

void ROS2Manager::set_cache_ttl(int ttl_seconds) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_ttl_seconds_ = std::max(1, std::min(60, ttl_seconds));
}

double ROS2Manager::get_cache_hit_ratio() const {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    if (cache_hits_ + cache_misses_ == 0) return 0.0;
    return static_cast<double>(cache_hits_) / 
           (cache_hits_ + cache_misses_);
}

void ROS2Manager::invalidate_cache() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    topics_cache_.reset();
    nodes_cache_.reset();
    services_cache_.reset();
}

std::vector<TopicInfo> ROS2Manager::discover_topics_() {
    std::vector<TopicInfo> topics;
    
    try {
        std::cerr << "[ROS2Manager] Starting topic discovery..." << std::endl;
        
        // Use ros2 topic list command with stderr suppressed
        // Source ROS2 environment to ensure ros2 command is available
        FILE* pipe = popen("bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null && ros2 topic list -t 2>/dev/null'", "r");
        if (!pipe) {
            std::cerr << "[ROS2Manager] Failed to execute ros2 topic list" << std::endl;
            return topics;
        }

        char buffer[512];
        int topic_count = 0;
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            std::string line(buffer);
            // Remove newline
            line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
            line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
            
            if (line.empty()) continue;
            
            // Parse "topic_name [msg_type]" format
            size_t bracket_pos = line.find('[');
            if (bracket_pos != std::string::npos) {
                std::string topic_name = line.substr(0, bracket_pos);
                // Trim trailing whitespace
                size_t end_pos = topic_name.find_last_not_of(" \t");
                if (end_pos != std::string::npos) {
                    topic_name = topic_name.substr(0, end_pos + 1);
                }
                
                // Validate topic name is not empty after trimming
                if (topic_name.empty()) {
                    continue;
                }
                
                size_t end_bracket = line.find(']', bracket_pos);
                if (end_bracket == std::string::npos) {
                    std::cerr << "[ROS2Manager] Invalid topic format, missing closing bracket: " << line << std::endl;
                    continue;
                }
                
                std::string msg_type = line.substr(bracket_pos + 1, end_bracket - bracket_pos - 1);
                
                // Validate message type is not empty
                if (msg_type.empty()) {
                    std::cerr << "[ROS2Manager] Empty message type for topic: " << topic_name << std::endl;
                    continue;
                }
                
                std::cerr << "[ROS2Manager] Found topic: " << topic_name << " [" << msg_type << "]" << std::endl;
                
                TopicInfo info;
                info.name = topic_name;
                info.msg_type = msg_type;
                info.publisher_count = 0;  // Will be updated by topic info command
                info.subscription_count = 0;
                
                topics.push_back(info);
                topic_count++;
            }
        }
        int pipe_status = pclose(pipe);
        if (pipe_status != 0) {
            std::cerr << "[ROS2Manager] ros2 topic list failed with status: " << pipe_status << std::endl;
        }
        
        std::cerr << "[ROS2Manager] Total topics discovered: " << topic_count << std::endl;
        
        // Get publisher/subscriber counts for each topic
        for (auto& topic : topics) {
            std::string cmd = "bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null && ros2 topic info " + topic.name + " 2>/dev/null | grep -E \"Publishers|Subscribers\"'";
            FILE* info_pipe = popen(cmd.c_str(), "r");
            if (info_pipe) {
                char info_buffer[256];
                while (fgets(info_buffer, sizeof(info_buffer), info_pipe) != nullptr) {
                    std::string info_line(info_buffer);
                    try {
                        if (info_line.find("Publishers:") != std::string::npos) {
                            // Parse "Publishers:      1"
                            size_t count_pos = info_line.find_last_not_of("0123456789");
                            if (count_pos != std::string::npos) {
                                std::string count_str = info_line.substr(count_pos + 1);
                                // Trim whitespace from count string
                                count_str.erase(std::remove(count_str.begin(), count_str.end(), '\n'), count_str.end());
                                count_str.erase(std::remove(count_str.begin(), count_str.end(), '\r'), count_str.end());
                                if (!count_str.empty()) {
                                    int count = std::stoi(count_str);
                                    topic.publisher_count = count;
                                }
                            }
                        } else if (info_line.find("Subscribers:") != std::string::npos) {
                            size_t count_pos = info_line.find_last_not_of("0123456789");
                            if (count_pos != std::string::npos) {
                                std::string count_str = info_line.substr(count_pos + 1);
                                // Trim whitespace from count string
                                count_str.erase(std::remove(count_str.begin(), count_str.end(), '\n'), count_str.end());
                                count_str.erase(std::remove(count_str.begin(), count_str.end(), '\r'), count_str.end());
                                if (!count_str.empty()) {
                                    int count = std::stoi(count_str);
                                    topic.subscription_count = count;
                                }
                            }
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "[ROS2Manager] Error parsing topic info for " << topic.name << ": " << e.what() << std::endl;
                    }
                }
                int info_pipe_status = pclose(info_pipe);
                if (info_pipe_status != 0) {
                    std::cerr << "[ROS2Manager] ros2 topic info failed for " << topic.name << " with status: " << info_pipe_status << std::endl;
                }
            } else {
                std::cerr << "[ROS2Manager] Failed to execute topic info command for " << topic.name << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[ROS2Manager] Exception during topic discovery: " << e.what() << std::endl;
    }

    return topics;
}

std::vector<NodeInfo> ROS2Manager::discover_nodes_() {
    std::vector<NodeInfo> nodes;
    
    try {
        FILE* pipe = popen("bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null && ros2 node list 2>/dev/null'", "r");
        if (!pipe) {
            std::cerr << "[ROS2Manager] Failed to execute ros2 node list" << std::endl;
            return nodes;
        }

        char buffer[256];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            std::string node_name(buffer);
            node_name.erase(
                std::remove(node_name.begin(), node_name.end(), '\n'),
                node_name.end());
            node_name.erase(
                std::remove(node_name.begin(), node_name.end(), '\r'),
                node_name.end());

            if (!node_name.empty()) {
                NodeInfo info;
                info.name = node_name;
                // Extract namespace from node name (e.g., "/namespace/nodename" -> "/namespace")
                size_t last_slash = node_name.rfind('/');
                if (last_slash != std::string::npos && last_slash > 0) {
                    info.namespace_ = node_name.substr(0, last_slash);
                } else {
                    info.namespace_ = "/";
                }
                
                // Get node info to list publications and subscriptions
                std::string cmd = "bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null && ros2 node info " + node_name + " 2>/dev/null'";
                FILE* info_pipe = popen(cmd.c_str(), "r");
                if (info_pipe) {
                    char info_buffer[512];
                    bool parsing_pubs = false;
                    bool parsing_subs = false;
                    
                    while (fgets(info_buffer, sizeof(info_buffer), info_pipe) != nullptr) {
                        std::string info_line(info_buffer);
                        
                        if (info_line.find("Subscriptions:") != std::string::npos) {
                            parsing_pubs = false;
                            parsing_subs = true;
                            continue;
                        } else if (info_line.find("Publications:") != std::string::npos) {
                            parsing_pubs = true;
                            parsing_subs = false;
                            continue;
                        } else if (info_line.find("Services:") != std::string::npos) {
                            parsing_pubs = false;
                            parsing_subs = false;
                        }
                        
                        // Parse topic lines (they start with spaces and have format "    /topic_name [type]")
                        if ((parsing_pubs || parsing_subs) && info_line[0] == ' ') {
                            info_line.erase(0, info_line.find_first_not_of(" \t"));
                            if (info_line.empty()) continue;
                            
                            // Remove the type suffix if present
                            size_t bracket_pos = info_line.find('[');
                            if (bracket_pos != std::string::npos) {
                                info_line = info_line.substr(0, bracket_pos);
                            }
                            // Trim trailing whitespace
                            size_t end_pos = info_line.find_last_not_of(" \t");
                            if (end_pos != std::string::npos) {
                                info_line = info_line.substr(0, end_pos + 1);
                            }
                            
                            if (!info_line.empty()) {
                                if (parsing_pubs) {
                                    info.publications.push_back(info_line);
                                } else if (parsing_subs) {
                                    info.subscriptions.push_back(info_line);
                                }
                            }
                        }
                    }
                    int info_pipe_status = pclose(info_pipe);
                    if (info_pipe_status != 0) {
                        std::cerr << "[ROS2Manager] ros2 node info failed for " << node_name << " with status: " << info_pipe_status << std::endl;
                    }
                } else {
                    std::cerr << "[ROS2Manager] Failed to execute node info command for " << node_name << std::endl;
                }
                
                nodes.push_back(info);
            }
        }
        int pipe_status = pclose(pipe);
        if (pipe_status != 0) {
            std::cerr << "[ROS2Manager] ros2 node list failed with status: " << pipe_status << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[ROS2Manager] Exception during node discovery: " << e.what() << std::endl;
    }

    return nodes;
}

std::vector<ServiceInfo> ROS2Manager::discover_services_() {
    std::vector<ServiceInfo> services;
    
    try {
        FILE* pipe = popen("bash -c 'source /opt/ros/humble/setup.bash 2>/dev/null && ros2 service list -t 2>/dev/null'", "r");
        if (!pipe) {
            std::cerr << "[ROS2Manager] Failed to execute ros2 service list" << std::endl;
            return services;
        }

        char buffer[512];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            std::string line(buffer);
            line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
            line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

            if (line.empty()) continue;
            
            // Parse "service_name [srv_type]" format
            size_t bracket_pos = line.find('[');
            if (bracket_pos != std::string::npos) {
                std::string service_name = line.substr(0, bracket_pos);
                // Trim trailing whitespace
                size_t end_pos = service_name.find_last_not_of(" \t");
                if (end_pos != std::string::npos) {
                    service_name = service_name.substr(0, end_pos + 1);
                }
                
                if (service_name.empty()) {
                    continue;
                }
                
                size_t end_bracket = line.find(']', bracket_pos);
                if (end_bracket == std::string::npos) {
                    std::cerr << "[ROS2Manager] Invalid service format, missing closing bracket: " << line << std::endl;
                    continue;
                }
                
                std::string srv_type = line.substr(bracket_pos + 1, end_bracket - bracket_pos - 1);
                
                if (srv_type.empty()) {
                    std::cerr << "[ROS2Manager] Empty service type for service: " << service_name << std::endl;
                    continue;
                }
                
                ServiceInfo info;
                info.name = service_name;
                info.service_type = srv_type;
                
                // TODO: Get list of servers providing this service
                // This would require ros2 service call or similar
                
                services.push_back(info);
            }
        }
        int pipe_status = pclose(pipe);
        if (pipe_status != 0) {
            std::cerr << "[ROS2Manager] ros2 service list failed with status: " << pipe_status << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[ROS2Manager] Exception during service discovery: " << e.what() << std::endl;
    }

    return services;
}

}  // namespace ros2_dashboard
