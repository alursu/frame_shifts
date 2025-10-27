#ifndef OPTICAL_FLOW_MAVLINK_H
#define OPTICAL_FLOW_MAVLINK_H

#include <string>
#include <map>
#include <vector>
#include <optional>
#include <any>
#include <memory>

// Forward declarations
class OpticalFlowMAVLink {
public:
    // Type definitions for return values
    using ResultDict = std::map<std::string, std::any>;
    
    // Constructor
    OpticalFlowMAVLink();
    ~OpticalFlowMAVLink();
    
    // Public API functions
    ResultDict send_optical_flow_msg(int sysid,
                                     int flow_x,
                                     int flow_y,
                                     float flow_comp_m_x,
                                     float flow_comp_m_y,
                                     int quality,
                                     float ground_distance,
                                     float flow_rate_x,
                                     float flow_rate_y);
    
    ResultDict send_statustext_msg(int sysid,
                                   const std::string& text,
                                   const std::string& severity = "MAV_SEVERITY_INFO",
                                   int message_id = 0,
                                   int chunk_seq = 0);
    
    ResultDict get_gimbal_attitude(int sysid);
    
    ResultDict request_gimbal_attitude_status(int sysid, float interval_hz);
    
    ResultDict send_set_message_interval(int sysid,
                                        int message_id,
                                        float interval_hz);

private:
    // Helper function for HTTP POST
    std::optional<std::string> post_to_mav2rest(const std::string& url,
                                                const std::string& data);
};

#endif // OPTICAL_FLOW_MAVLINK_H





// ```cpp
// // MAVLink OpticalFlow Script.
// //
// // This script sends out OpticalFlow detections using the MAVLink protocol to
// // an ArduPilot/PixHawk controller for position control using your OpenMV Cam.
// //
// // P8 = TXD

// #include <cstdint>
// #include <cstring>
// #include <cstdio>

// // OpenMV C++ API Headers
// #include "sensor.h"
// #include "uart.h"
// #include "led.h"
// #include "framebuffer.h"
// #include "py_time.h"

// const int UART_BAUDRATE = 115200;
// const uint8_t MAV_system_id = 1;
// const uint8_t MAV_component_id = 0x54;
// uint8_t packet_sequence = 0;

// // Below 0.1 or so (YMMV) and the results are just noise.
// const float MAV_OPTICAL_FLOW_confidence_threshold = 0.1f;

// // LED control
// LED* led = nullptr;
// int led_state = 0;

// void update_led() {
//     led_state = led_state + 1;
//     if (led_state == 5) {
//         led->on();
//     } else if (led_state >= 10) {
//         led->off();
//         led_state = 0;
//     }
// }

// // Link Setup
// UART* uart = nullptr;

// // https://github.com/mavlink/c_library_v1/blob/master/checksum.h
// uint16_t checksum(const uint8_t* data, size_t len, uint8_t extra) {
//     uint16_t output = 0xFFFF;
//     for (size_t i = 0; i < len; i++) {
//         uint8_t tmp = data[i] ^ (output & 0xFF);
//         tmp = (tmp ^ (tmp << 4)) & 0xFF;
//         output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
//     }
//     uint8_t tmp = extra ^ (output & 0xFF);
//     tmp = (tmp ^ (tmp << 4)) & 0xFF;
//     output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
//     return output;
// }

// const uint8_t MAV_OPTICAL_FLOW_message_id = 100;
// const uint8_t MAV_OPTICAL_FLOW_id = 0;  // unused
// const uint8_t MAV_OPTICAL_FLOW_extra_crc = 175;

// // http://mavlink.org/messages/common#OPTICAL_FLOW
// https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_optical_flow.h
// void send_optical_flow_packet(int x, int y, float c) {
//     // Pack the 26-byte payload
//     uint8_t payload[26];
    
//     // <qfffhhbb format: uint64_t, float, float, float, int16_t, int16_t, int8_t, int8_t
//     uint64_t time_usec = 0;
//     float flow_comp_m_x = 0.0f;
//     float flow_comp_m_y = 0.0f;
//     float ground_distance = 0.0f;
//     int16_t flow_x = static_cast<int16_t>(x);
//     int16_t flow_y = static_cast<int16_t>(y);
//     int8_t sensor_id = static_cast<int8_t>(MAV_OPTICAL_FLOW_id);
//     int8_t quality = static_cast<int8_t>(c * 255);
    
//     // Manual packing in little-endian format
//     size_t offset = 0;
//     memcpy(payload + offset, &time_usec, sizeof(uint64_t)); offset += sizeof(uint64_t);
//     memcpy(payload + offset, &flow_comp_m_x, sizeof(float)); offset += sizeof(float);
//     memcpy(payload + offset, &flow_comp_m_y, sizeof(float)); offset += sizeof(float);
//     memcpy(payload + offset, &ground_distance, sizeof(float)); offset += sizeof(float);
//     memcpy(payload + offset, &flow_x, sizeof(int16_t)); offset += sizeof(int16_t);
//     memcpy(payload + offset, &flow_y, sizeof(int16_t)); offset += sizeof(int16_t);
//     memcpy(payload + offset, &sensor_id, sizeof(int8_t)); offset += sizeof(int8_t);
//     memcpy(payload + offset, &quality, sizeof(int8_t)); offset += sizeof(int8_t);
    
//     // Pack the MAVLink header + payload (31 bytes)
//     uint8_t header_payload[31];
//     header_payload[0] = 26;  // payload length
//     header_payload[1] = packet_sequence & 0xFF;
//     header_payload[2] = MAV_system_id;
//     header_payload[3] = MAV_component_id;
//     header_payload[4] = MAV_OPTICAL_FLOW_message_id;
//     memcpy(header_payload + 5, payload, 26);
    
//     // Calculate checksum
//     uint16_t crc = checksum(header_payload, 31, MAV_OPTICAL_FLOW_extra_crc);
    
//     // Build final packet (34 bytes total)
//     uint8_t packet[34];
//     packet[0] = 0xFE;  // MAVLink v1 start byte
//     memcpy(packet + 1, header_payload, 31);
//     memcpy(packet + 32, &crc, sizeof(uint16_t));
    
//     packet_sequence++;
//     uart->write(packet, 34);
//     update_led();
// }

// int main() {
//     // LED initialization
//     led = new LED(LED_BLUE);
    
//     // Link Setup
//     uart = new UART(3, UART_BAUDRATE, 1000);
    
//     sensor_reset();  // Reset and initialize the sensor.
//     sensor_set_pixformat(PIXFORMAT_RGB565);  // Set pixel format to RGB565 (or GRAYSCALE)
//     sensor_set_framesize(FRAMESIZE_B128X128);  // Set frame size to 64x32... (or 64x64)... B128X128  B64X64
//     sensor_skip_frames(2000);  // Wait for settings take effect.
    
//     Clock clock;  // Create a clock object to track the FPS.
    
//     // Take from the main frame buffer's RAM to allocate a second frame buffer.
//     // There's a lot more RAM in the frame buffer than in the MicroPython heap.
//     // However, after doing this you have a lot less RAM for some algorithms...
//     // So, be aware that it's a lot easier to get out of RAM issues now.
//     FrameBuffer* extra_fb = sensor_alloc_extra_fb(sensor_get_width(), sensor_get_height(), PIXFORMAT_RGB565);
//     FrameBuffer* current_fb = sensor_snapshot();
//     extra_fb->replace(current_fb);
    
//     while (true) {
//         clock.tick();  // Track elapsed milliseconds between snapshots().
//         FrameBuffer* img = sensor_snapshot();  // Take a picture and return the image.
        
//         Displacement displacement = extra_fb->find_displacement(img);
//         extra_fb->replace(img);
        
//         // Offset results are noisy without filtering so we drop some accuracy.
//         int sub_pixel_x = static_cast<int>(-displacement.x_translation() * 35);
//         int sub_pixel_y = static_cast<int>(displacement.y_translation() * 53);
        
//         send_optical_flow_packet(sub_pixel_x, sub_pixel_y, displacement.response());
        
//         printf("%+fx %+fy %f %f FPS\n", 
//                static_cast<float>(sub_pixel_x), 
//                static_cast<float>(sub_pixel_y), 
//                displacement.response(), 
//                clock.fps());
//     }
    
//     // Cleanup (unreachable but good practice)
//     delete extra_fb;
//     delete uart;
//     delete led;
    
//     return 0;
// }
// ```

// // Supporting type definitions and API wrappers for OpenMV C++ environment

// #ifndef OPENMV_CPP_API_H
// #define OPENMV_CPP_API_H

// // LED class wrapper
// enum LEDColor {
//     LED_RED,
//     LED_GREEN,
//     LED_BLUE
// };

// class LED {
// private:
//     LEDColor color;
    
// public:
//     LED(LEDColor c) : color(c) {
//         // Initialize LED hardware
//         led_init(color);
//     }
    
//     void on() {
//         led_set_state(color, true);
//     }
    
//     void off() {
//         led_set_state(color, false);
//     }
    
// private:
//     void led_init(LEDColor c);
//     void led_set_state(LEDColor c, bool state);
// };

// // UART class wrapper
// class UART {
// private:
//     int port;
//     int baudrate;
//     int timeout;
    
// public:
//     UART(int p, int baud, int timeout_ms) : port(p), baudrate(baud), timeout(timeout_ms) {
//         // Initialize UART hardware
//         uart_init(port, baudrate, timeout);
//     }
    
//     void write(const uint8_t* data, size_t len) {
//         uart_write_bytes(port, data, len);
//     }
    
// private:
//     void uart_init(int p, int baud, int timeout_ms);
//     void uart_write_bytes(int p, const uint8_t* data, size_t len);
// };

// // Displacement result structure
// struct Displacement {
// private:
//     float x_trans;
//     float y_trans;
//     float resp;
    
// public:
//     Displacement() : x_trans(0.0f), y_trans(0.0f), resp(0.0f) {}
    
//     Displacement(float x, float y, float r) : x_trans(x), y_trans(y), resp(r) {}
    
//     float x_translation() const {
//         return x_trans;
//     }
    
//     float y_translation() const {
//         return y_trans;
//     }
    
//     float response() const {
//         return resp;
//     }
// };

// // FrameBuffer class wrapper
// class FrameBuffer {
// private:
//     uint8_t* buffer;
//     int width;
//     int height;
//     int pixformat;
    
// public:
//     FrameBuffer(int w, int h, int fmt) : width(w), height(h), pixformat(fmt) {
//         size_t size = w * h * 2;  // RGB565 is 2 bytes per pixel
//         buffer = new uint8_t[size];
//     }
    
//     ~FrameBuffer() {
//         delete[] buffer;
//     }
    
//     void replace(FrameBuffer* other) {
//         size_t size = width * height * 2;
//         memcpy(buffer, other->buffer, size);
//     }
    
//     Displacement find_displacement(FrameBuffer* other) {
//         // Call OpenMV's displacement calculation function
//         float x_trans, y_trans, resp;
//         imlib_find_displacement(buffer, other->buffer, width, height, &x_trans, &y_trans, &resp);
//         return Displacement(x_trans, y_trans, resp);
//     }
    
// private:
//     void imlib_find_displacement(uint8_t* fb1, uint8_t* fb2, int w, int h, 
//                                   float* x_trans, float* y_trans, float* response);
// };

// // Clock class for FPS tracking
// class Clock {
// private:
//     uint32_t last_tick;
//     uint32_t frame_count;
//     uint32_t fps_start_time;
//     float current_fps;
    
// public:
//     Clock() : last_tick(0), frame_count(0), fps_start_time(0), current_fps(0.0f) {
//         last_tick = millis();
//         fps_start_time = last_tick;
//     }
    
//     void tick() {
//         last_tick = millis();
//         frame_count++;
        
//         uint32_t elapsed = last_tick - fps_start_time;
//         if (elapsed >= 1000) {
//             current_fps = (frame_count * 1000.0f) / elapsed;
//             frame_count = 0;
//             fps_start_time = last_tick;
//         }
//     }
    
//     float fps() const {
//         return current_fps;
//     }
    
// private:
//     uint32_t millis();
// };

// // Sensor API wrapper functions
// enum PixFormat {
//     PIXFORMAT_RGB565,
//     PIXFORMAT_GRAYSCALE
// };

// enum FrameSize {
//     FRAMESIZE_B64X64,
//     FRAMESIZE_B128X128
// };

// void sensor_reset();
// void sensor_set_pixformat(PixFormat fmt);
// void sensor_set_framesize(FrameSize size);
// void sensor_skip_frames(int time_ms);
// FrameBuffer* sensor_snapshot();
// FrameBuffer* sensor_alloc_extra_fb(int width, int height, PixFormat fmt);
// int sensor_get_width();
// int sensor_get_height();

// #endif // OPENMV_CPP_API_H
// ```
