// /**
//  * Optical Flow MAVLink Message Module
//  *
//  * This module handles sending OPTICAL_FLOW and STATUSTEXT MAVLink messages to the vehicle
//  * via BlueOS MAV2Rest API interface.
//  */

#include "mavlinkinterface.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cstring>
#include <stdexcept>
// #include <curl/curl.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Logger class for mimicking Python logging
class Logger {
public:
    enum Level {
        DEBUG,
        INFO,
        WARNING,
        ERROR
    };
    
    Logger(const std::string& name) : name_(name), level_(INFO) {}
    
    void setLevel(Level level) {
        level_ = level;
    }
    
    void debug(const std::string& message) {
        if (level_ <= DEBUG) {
            log("DEBUG", message);
        }
    }
    
    void info(const std::string& message) {
        if (level_ <= INFO) {
            log("INFO", message);
        }
    }
    
    void warning(const std::string& message) {
        if (level_ <= WARNING) {
            log("WARNING", message);
        }
    }
    
    void error(const std::string& message) {
        if (level_ <= ERROR) {
            log("ERROR", message);
        }
    }
    
private:
    void log(const std::string& level, const std::string& message) {
        std::cerr << "[" << level << "] " << name_ << ": " << message << std::endl;
    }
    
    std::string name_;
    Level level_;
};

// Get logger
static Logger logger("opticalflow");

// MAV2Rest endpoint
static const std::string MAV2REST_ENDPOINT = "http://host.docker.internal:6040";

// MAVLink component ID
static const int MAV_COMP_ID_ONBOARD_COMPUTER = 191;  // Component ID for onboard computer systems

// COMMAND_LONG message template for SET_MESSAGE_INTERVAL
static const char* COMMAND_LONG_SET_MESSAGE_INTERVAL_TEMPLATE = R"({
  "header": {
    "system_id": {sysid},
    "component_id": {component_id},
    "sequence": 0
  },
  "message": {
    "type": "COMMAND_LONG",
    "target_system": {target_system},
    "target_component": {target_component},
    "command": {
      "type": "MAV_CMD_SET_MESSAGE_INTERVAL"
    },
    "confirmation": 0,
    "param1": {message_id},
    "param2": {interval_us},
    "param3": {param3},
    "param4": {param4},
    "param5": {param5},
    "param6": {param6},
    "param7": {response_target}
  }
})";

// OPTICAL_FLOW message template
static const char* OPTICAL_FLOW_TEMPLATE = R"({
  "header": {
    "system_id": {sysid},
    "component_id": {component_id},
    "sequence": 0
  },
  "message": {
    "type": "OPTICAL_FLOW",
    "time_usec": {time_usec},
    "sensor_id": {sensor_id},
    "flow_x": {flow_x},
    "flow_y": {flow_y},
    "flow_comp_m_x": {flow_comp_m_x},
    "flow_comp_m_y": {flow_comp_m_y},
    "quality": {quality},
    "ground_distance": {ground_distance},
    "flow_rate_x": {flow_rate_x},
    "flow_rate_y": {flow_rate_y}
  }
})";

// STATUSTEXT message template
static const char* STATUSTEXT_TEMPLATE = R"({
  "header": {
    "system_id": {sysid},
    "component_id": {component_id},
    "sequence": 0
  },
  "message": {
    "type": "STATUSTEXT",
    "severity": {
      "type": "{severity}"
    },
    "text": {text_array},
    "id": {id},
    "chunk_seq": {chunk_seq}
  }
})";

// Helper function to replace placeholders in template strings
static std::string format_template(const std::string& template_str,
                                   const std::map<std::string, std::string>& replacements) {
    std::string result = template_str;
    for (const auto& [key, value] : replacements) {
        std::string placeholder = "{" + key + "}";
        size_t pos = 0;
        while ((pos = result.find(placeholder, pos)) != std::string::npos) {
            result.replace(pos, placeholder.length(), value);
            pos += value.length();
        }
    }
    return result;
}

// Callback function for libcurl to capture response data
static size_t write_callback(void* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

// send mavlink message using MAV2Rest
std::optional<std::string> OpticalFlowMAVLink::post_to_mav2rest(const std::string& url,
                                                                 const std::string& data) {
    /**
     * Sends a POST request to MAV2Rest with JSON data
     * Returns response text if successful, None otherwise
     */
    try {
        CURL* curl = curl_easy_init();
        if (!curl) {
            logger.error("post_to_mav2rest: error : " + url + ": Failed to initialize CURL");
            return std::nullopt;
        }
        
        std::string response_string;
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        
        // Set CURL options
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
        
        // Perform the request
        CURLcode res = curl_easy_perform(curl);
        
        // Cleanup
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
        
        if (res != CURLE_OK) {
            logger.error("post_to_mav2rest: error : " + url + ": " + std::string(curl_easy_strerror(res)));
            return std::nullopt;
        }
        
        return response_string;
        
    } catch (const std::exception& error) {
        logger.error("post_to_mav2rest: error : " + url + ": " + std::string(error.what()));
        return std::nullopt;
    }
}

// Constructor
OpticalFlowMAVLink::OpticalFlowMAVLink() {
    // Initialize CURL library
    curl_global_init(CURL_GLOBAL_DEFAULT);
}

// Destructor
OpticalFlowMAVLink::~OpticalFlowMAVLink() {
    // Cleanup CURL library
    curl_global_cleanup();
}

// Low level function to send OPTICAL_FLOW MAVLink message
OpticalFlowMAVLink::ResultDict OpticalFlowMAVLink::send_optical_flow_msg(
    int sysid,
    int flow_x,
    int flow_y,
    float flow_comp_m_x,
    float flow_comp_m_y,
    int quality,
    float ground_distance,
    float flow_rate_x,
    float flow_rate_y) {
    /**
     * Send OPTICAL_FLOW MAVLink message
     *
     * Args:
     *     sysid: System ID to send message (normally 1)
     *     flow_x: Flow in x-sensor direction in dpix
     *     flow_y: Flow in y-sensor direction in dpiy
     *     flow_comp_m_x: Flow in x-axis in ground plane in meters/second
     *     flow_comp_m_y: Flow in y-axis in ground plane in meters/second
     *     quality: Optical flow quality (0=bad, 255=maximum quality)
     *     ground_distance: Ground distance in meters, negative if unknown
     *     flow_rate_x: Flow rate about X axis in radians/second
     *     flow_rate_y: Flow rate about Y axis in radians/second
     *
     * Returns:
     *     Dictionary with send results
     */
    
    // logging prefix for all messages from this function
    std::string logging_prefix_str = "send_optical_flow_msg:";
    
    try {
        // Get current time in microseconds since UNIX epoch
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        int64_t time_usec = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
        
        // Format the OPTICAL_FLOW message
        std::map<std::string, std::string> replacements;
        replacements["sysid"] = std::to_string(sysid);
        replacements["component_id"] = std::to_string(MAV_COMP_ID_ONBOARD_COMPUTER);
        replacements["time_usec"] = std::to_string(time_usec);
        replacements["sensor_id"] = "0";
        replacements["flow_x"] = std::to_string(static_cast<int>(flow_x));  // Flow in x-sensor direction in dpix
        replacements["flow_y"] = std::to_string(static_cast<int>(flow_y));  // Flow in y-sensor direction in dpiy
        replacements["flow_comp_m_x"] = std::to_string(flow_comp_m_x);  // Flow in x-sensor direction in m/s, angular-speed compensated
        replacements["flow_comp_m_y"] = std::to_string(flow_comp_m_y);  // Flow in y-sensor direction in m/s, angular-speed compensated
        replacements["quality"] = std::to_string(quality);  // Optical flow quality / confidence. 0: bad, 255: maximum quality
        replacements["ground_distance"] = std::to_string(ground_distance);  // Ground distance. Positive value: distance known. Negative value: Unknown distance
        replacements["flow_rate_x"] = std::to_string(flow_rate_x);  // Flow rate about X axis in radians/second
        replacements["flow_rate_y"] = std::to_string(flow_rate_y);  // Flow rate about Y axis in radians/second
        
        std::string optical_flow_data = format_template(OPTICAL_FLOW_TEMPLATE, replacements);
        
        // Send message via MAV2Rest
        std::string url = MAV2REST_ENDPOINT + "/mavlink";
        auto response = post_to_mav2rest(url, optical_flow_data);
        
        if (response.has_value()) {
            std::ostringstream oss;
            oss << logging_prefix_str << " OPTICAL_FLOW sent with SysID " << sysid 
                << " CompID " << MAV_COMP_ID_ONBOARD_COMPUTER 
                << " flow_rate_x=" << std::fixed << std::setprecision(4) << flow_rate_x 
                << " rad/s, flow_rate_y=" << std::fixed << std::setprecision(4) << flow_rate_y 
                << " rad/s";
            logger.debug(oss.str());
            
            ResultDict result;
            result["success"] = true;
            result["message"] = "OPTICAL_FLOW message sent successfully with SysID " + 
                               std::to_string(sysid) + " CompID " + 
                               std::to_string(MAV_COMP_ID_ONBOARD_COMPUTER);
            result["time_usec"] = time_usec;
            result["flow_rate_x"] = flow_rate_x;
            result["flow_rate_y"] = flow_rate_y;
            result["quality"] = quality;
            result["ground_distance"] = ground_distance;
            result["sysid"] = sysid;
            result["response"] = response.value();
            return result;
        } else {
            logger.error(logging_prefix_str + " failed to send OPTICAL_FLOW");
            ResultDict result;
            result["success"] = false;
            result["message"] = std::string("MAV2Rest returned no response");
            result["network_error"] = true;
            return result;
        }
        
    } catch (const std::exception& e) {
        logger.error(logging_prefix_str + " unexpected error " + std::string(e.what()));
        ResultDict result;
        result["success"] = false;
        result["message"] = std::string("Unexpected error: ") + std::string(e.what());
        result["unexpected_error"] = true;
        return result;
    }
}

// Low level function to send STATUSTEXT MAVLink message
OpticalFlowMAVLink::ResultDict OpticalFlowMAVLink::send_statustext_msg(
    int sysid,
    const std::string& text,
    const std::string& severity,
    int message_id,
    int chunk_seq) {
    /**
     * Send STATUSTEXT MAVLink message
     *
     * Args:
     *     sysid: System ID to send message (normally 1)
     *     text: Status text message (max 50 characters)
     *     severity: Message severity level (MAV_SEVERITY_EMERGENCY, MAV_SEVERITY_ALERT,
     *              MAV_SEVERITY_CRITICAL, MAV_SEVERITY_ERROR, MAV_SEVERITY_WARNING,
     *              MAV_SEVERITY_NOTICE, MAV_SEVERITY_INFO, MAV_SEVERITY_DEBUG)
     *     message_id: Unique (or wrap around) id for this message (0 for auto)
     *     chunk_seq: Allows the safe construction of larger messages from chunks
     *
     * Returns:
     *     Dictionary with send results
     */
    
    // logging prefix for all messages from this function
    std::string logging_prefix_str = "send_statustext_msg:";
    
    try {
        // Truncate text to 50 characters (MAVLink STATUSTEXT limit)
        std::string truncated_text = text;
        if (truncated_text.length() > 50) {
            truncated_text = truncated_text.substr(0, 50);
            logger.warning(logging_prefix_str + " text truncated to 50 characters");
        }
        
        // Convert text string to character array as required by MAV2Rest
        // MAVLink STATUSTEXT expects a 50-character array, null-terminated
        std::vector<std::string> text_chars;
        for (char c : truncated_text) {
            text_chars.push_back(std::string(1, c));
        }
        
        // Pad with null terminators to make it 50 characters total
        while (text_chars.size() < 50) {
            text_chars.push_back("\\u0000");
        }
        
        // Convert to JSON array format
        json text_array_json = json::array();
        for (const auto& c : text_chars) {
            if (c == "\\u0000") {
                text_array_json.push_back("\u0000");
            } else {
                text_array_json.push_back(c);
            }
        }
        std::string text_array = text_array_json.dump();
        
        // Format the STATUSTEXT message
        std::map<std::string, std::string> replacements;
        replacements["sysid"] = std::to_string(sysid);
        replacements["component_id"] = std::to_string(MAV_COMP_ID_ONBOARD_COMPUTER);
        replacements["severity"] = severity;
        replacements["text_array"] = text_array;
        replacements["id"] = std::to_string(message_id);
        replacements["chunk_seq"] = std::to_string(chunk_seq);
        
        std::string statustext_data = format_template(STATUSTEXT_TEMPLATE, replacements);
        
        // Send message via MAV2Rest
        std::string url = MAV2REST_ENDPOINT + "/mavlink";
        auto response = post_to_mav2rest(url, statustext_data);
        
        if (response.has_value()) {
            logger.debug(logging_prefix_str + " STATUSTEXT sent with SysID " + 
                        std::to_string(sysid) + " CompID " + 
                        std::to_string(MAV_COMP_ID_ONBOARD_COMPUTER) + ": '" + 
                        truncated_text + "'");
            
            ResultDict result;
            result["success"] = true;
            result["message"] = "STATUSTEXT message sent successfully with SysID " + 
                               std::to_string(sysid) + " CompID " + 
                               std::to_string(MAV_COMP_ID_ONBOARD_COMPUTER);
            result["text"] = truncated_text;
            result["severity"] = severity;
            result["sysid"] = sysid;
            result["response"] = response.value();
            return result;
        } else {
            logger.error(logging_prefix_str + " failed to send STATUSTEXT");
            ResultDict result;
            result["success"] = false;
            result["message"] = std::string("MAV2Rest returned no response");
            result["network_error"] = true;
            return result;
        }
        
    } catch (const std::exception& e) {
        logger.error(logging_prefix_str + " unexpected error " + std::string(e.what()));
        ResultDict result;
        result["success"] = false;
        result["message"] = std::string("Unexpected error: ") + std::string(e.what());
        result["unexpected_error"] = true;
        return result;
    }
}

// Retrieve the latest GIMBAL_DEVICE_ATTITUDE_STATUS message via MAV2Rest
OpticalFlowMAVLink::ResultDict OpticalFlowMAVLink::get_gimbal_attitude(int sysid) {
    /**
     * Retrieve the latest GIMBAL_DEVICE_ATTITUDE_STATUS message via MAV2Rest
     *
     * Args:
     *     sysid: System ID to query gimbal attitude from
     *
     * Returns:
     *     Dictionary with gimbal attitude data or error information
     */
    
    // logging prefix for all messages from this function
    std::string logging_prefix_str = "get_gimbal_attitude:";
    
    try {
        // Request the latest GIMBAL_DEVICE_ATTITUDE_STATUS message from the autopilot
        int compid = 1;  // autopilot component ID
        std::string url = MAV2REST_ENDPOINT + "/mavlink/vehicles/" + 
                         std::to_string(sysid) + "/components/" + 
                         std::to_string(compid) + "/messages/GIMBAL_DEVICE_ATTITUDE_STATUS";
        
        CURL* curl = curl_easy_init();
        if (!curl) {
            logger.error(logging_prefix_str + " Failed to initialize CURL");
            ResultDict result;
            result["success"] = false;
            result["message"] = std::string("Failed to initialize CURL");
            result["network_error"] = true;
            return result;
        }
        
        std::string response_string;
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        
        // Set CURL options
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);
        
        // Perform the request
        CURLcode res = curl_easy_perform(curl);
        
        long response_code = 0;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
        
        // Cleanup
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
        
        if (res != CURLE_OK) {
            if (res == CURLE_HTTP_RETURNED_ERROR || response_code == 404) {
                logger.warning(logging_prefix_str + " GIMBAL_DEVICE_ATTITUDE_STATUS not found");
                ResultDict result;
                result["success"] = false;
                result["message"] = std::string("Gimbal not found or not publishing attitude data");
                result["error"] = std::string("Gimbal not available");
                return result;
            } else {
                logger.error(logging_prefix_str + " Network error retrieving gimbal attitude: " + 
                           std::string(curl_easy_strerror(res)));
                ResultDict result;
                result["success"] = false;
                result["message"] = std::string("Network error: ") + std::string(curl_easy_strerror(res));
                result["network_error"] = true;
                return result;
            }
        }
        
        if (response_code == 200) {
            // Parse the response (MAV2Rest returns JSON)
            json data = json::parse(response_string);
            
            // Extract attitude information from the message
            if (data.contains("message")) {
                json msg = data["message"];
                
                // Extract quaternion components
                std::vector<double> q = {1.0, 0.0, 0.0, 0.0};
                if (msg.contains("q")) {
                    q = msg["q"].get<std::vector<double>>();
                }
                
                // log attitude as quaternion
                std::ostringstream oss;
                oss << logging_prefix_str << " q0:" << std::fixed << std::setprecision(4) << q[0]
                    << ", q1:" << std::fixed << std::setprecision(4) << q[1]
                    << ", q2:" << std::fixed << std::setprecision(5) << q[2]
                    << ", q3:" << std::fixed << std::setprecision(4) << q[3];
                logger.debug(oss.str());
                
                ResultDict result;
                result["success"] = true;
                result["message"] = std::string("Gimbal attitude retrieved successfully");
                
                std::map<std::string, double> quaternion;
                quaternion["w"] = q[0];
                quaternion["x"] = q[1];
                quaternion["y"] = q[2];
                quaternion["z"] = q[3];
                result["quaternion"] = quaternion;
                
                return result;
            } else {
                logger.warning(logging_prefix_str + " could not parse GIMBAL_DEVICE_ATTITUDE_STATUS message");
                ResultDict result;
                result["success"] = false;
                result["message"] = std::string("Invalid gimbal attitude message format");
                result["raw_response"] = response_string;
                return result;
            }
        } else if (response_code == 404) {
            logger.warning(logging_prefix_str + " GIMBAL_DEVICE_ATTITUDE_STATUS not found");
            ResultDict result;
            result["success"] = false;
            result["message"] = std::string("Gimbal not found or not publishing attitude data");
            result["error"] = std::string("Gimbal not available");
            return result;
        } else {
            logger.warning(logging_prefix_str + " HTTP error " + 
                          std::to_string(response_code) + " retrieving gimbal attitude");
            ResultDict result;
            result["success"] = false;
            result["message"] = "HTTP error " + std::to_string(response_code);
            result["http_error"] = true;
            return result;
        }
        
    } catch (const json::exception& e) {
        logger.error(logging_prefix_str + " JSON parsing error: " + std::string(e.what()));
        ResultDict result;
        result["success"] = false;
        result["message"] = std::string("JSON parsing error: ") + std::string(e.what());
        result["unexpected_error"] = true;
        return result;
    } catch (const std::exception& e) {
        logger.error(logging_prefix_str + " could not retrieve GIMBAL_DEVICE_ATTITUDE_STATUS: " + 
                    std::string(e.what()));
        ResultDict result;
        result["success"] = false;
        result["message"] = std::string("Unexpected error: ") + std::string(e.what());
        result["unexpected_error"] = true;
        return result;
    }
}

// request GIMBAL_DEVICE_ATTITUDE_STATUS messages at specified rate
OpticalFlowMAVLink::ResultDict OpticalFlowMAVLink::request_gimbal_attitude_status(
    int sysid, 
    float interval_hz) {
    /**
     * Request GIMBAL_DEVICE_ATTITUDE_STATUS messages at specified rate
     *
     * Args:
     *     sysid: System ID to send message to
     *     interval_hz: Frequency in Hz (1.0 = 1Hz, 0 = disable)
     *
     * Returns:
     *     Dictionary with send results
     */
    
    std::ostringstream oss;
    oss << "request_gimbal_attitude_status: requesting GIMBAL_DEVICE_ATTITUDE_STATUS at " 
        << interval_hz << "Hz";
    logger.debug(oss.str());
    
    return send_set_message_interval(
        sysid,
        285,         // GIMBAL_DEVICE_ATTITUDE_STATUS
        interval_hz
    );
}

// send SET_MESSAGE_INTERVAL command
OpticalFlowMAVLink::ResultDict OpticalFlowMAVLink::send_set_message_interval(
    int sysid,          // target system id
    int message_id,     // MAVlink message id
    float interval_hz) {
    /**
     * Send COMMAND_LONG with MAV_CMD_SET_MESSAGE_INTERVAL to request specific message at given rate
     *
     * Args:
     *     sysid: System ID to send message to (e.g 1 for autopilot)
     *     message_id: MAVLink message ID to request (e.g 285 for GIMBAL_DEVICE_ATTITUDE_STATUS)
     *     interval_hz: Frequency in Hz (-1 = disable, 0 = request default rate, 1.0 = 1Hz)
     *
     * Returns:
     *     Dictionary with send results
     */
    std::string logging_prefix_str = "set_message_interval";
    
    try {
        // Convert frequency to interval in microseconds
        int interval_us;
        if (interval_hz < 0) {
            interval_us = -1;  // Disable the message
            logger.info(logging_prefix_str + ": disabling message " + std::to_string(message_id));
        } else if (interval_hz == 0) {
            interval_us = 0;  // Request default rate (0 = default)
            logger.info(logging_prefix_str + ": default rate for " + std::to_string(message_id));
        } else {
            interval_us = static_cast<int>(1000000 / interval_hz);  // Convert Hz to microseconds
            std::ostringstream oss;
            oss << logging_prefix_str << ": requesting message ID " << message_id 
                << " at " << interval_hz << "Hz (" << interval_us << "µs interval)";
            logger.info(oss.str());
        }
        
        // Format the COMMAND_LONG message with SET_MESSAGE_INTERVAL command
        int target_compid = 1;  // autopilot component ID
        std::map<std::string, std::string> replacements;
        replacements["sysid"] = std::to_string(sysid);
        replacements["component_id"] = std::to_string(MAV_COMP_ID_ONBOARD_COMPUTER);
        replacements["target_system"] = std::to_string(sysid);
        replacements["target_component"] = std::to_string(target_compid);  // autopilot component ID
        replacements["message_id"] = std::to_string(message_id);
        replacements["interval_us"] = std::to_string(interval_us);
        replacements["param3"] = "0.0";        // Unused
        replacements["param4"] = "0.0";        // Unused
        replacements["param5"] = "0.0";        // Unused
        replacements["param6"] = "0.0";        // Unused
        replacements["response_target"] = "1";  // requestor is target address of message stream
        
        std::string command_data = format_template(COMMAND_LONG_SET_MESSAGE_INTERVAL_TEMPLATE, 
                                                   replacements);
        
        // Send message via MAV2Rest
        std::string url = MAV2REST_ENDPOINT + "/mavlink";
        auto response = post_to_mav2rest(url, command_data);
        
        if (response.has_value()) {
            std::ostringstream oss;
            oss << logging_prefix_str << ": sent SET_MESSAGE_INTERVAL to sysid:" << sysid 
                << " compid:" << target_compid << " msg_id:" << message_id 
                << ", interval=" << interval_hz << "Hz)";
            logger.info(oss.str());
            
            ResultDict result;
            result["success"] = true;
            std::ostringstream msg_oss;
            msg_oss << "SET_MESSAGE_INTERVAL sent to sysid:" << sysid 
                    << " compid:" << target_compid << " msg_id:" << message_id 
                    << ", interval=" << interval_hz << "Hz)";
            result["message"] = msg_oss.str();
            result["message_id"] = message_id;
            result["interval_hz"] = interval_hz;
            result["interval_us"] = interval_us;
            result["target_component"] = target_compid;
            result["response"] = response.value();
            return result;
        } else {
            std::ostringstream oss;
            oss << logging_prefix_str << ": failed to send SET_MESSAGE_INTERVAL to sysid:" 
                << sysid << " compid:" << target_compid << " msg_id:" << message_id 
                << ", interval=" << interval_hz << "Hz)";
            logger.error(oss.str());
            
            ResultDict result;
            result["success"] = false;
            result["message"] = std::string("MAV2Rest returned no response");
            result["network_error"] = true;
            return result;
        }
        
    } catch (const std::exception& e) {
        logger.error(logging_prefix_str + ": failed to send SET_MESSAGE_INTERVAL: " + 
                    std::string(e.what()));
        ResultDict result;
        result["success"] = false;
        result["message"] = std::string("Unexpected error: ") + std::string(e.what());
        result["unexpected_error"] = true;
        return result;
    }
}

// main.cpp (Example usage)
// #include "optical_flow_mavlink.h"
// #include <iostream>

// int main() {
//     // Example usage of the OpticalFlowMAVLink class
//     OpticalFlowMAVLink mavlink;
    
//     // Example: Send optical flow message
//     auto result = mavlink.send_optical_flow_msg(
//         1,      // sysid
//         100,    // flow_x
//         -50,    // flow_y
//         0.5f,   // flow_comp_m_x
//         -0.3f,  // flow_comp_m_y
//         200,    // quality
//         2.5f,   // ground_distance
//         0.1f,   // flow_rate_x
//         -0.05f  // flow_rate_y
//     );
    
//     // Check result
//     try {
//         bool success = std::any_cast<bool>(result["success"]);
//         std::string message = std::any_cast<std::string>(result["message"]);
//         std::cout << "Success: " << success << ", Message: " << message << std::endl;
//     } catch (const std::bad_any_cast& e) {
//         std::cerr << "Error accessing result: " << e.what() << std::endl;
//     }
    
//     return 0;
// }