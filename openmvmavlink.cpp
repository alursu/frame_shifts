// MAVLink OpticalFlow Script.
//
// This script sends out OpticalFlow detections using the MAVLink protocol to
// an ArduPilot/PixHawk controller for position control using your OpenMV Cam.
//
// P8 = TXD

#include <cstdint>
#include <cstring>
#include <cstdio>
#include "openmvmavlink.hpp"

// OpenMV C++ API Headers
// #include "sensor.h"
// #include "uart.h"
// #include "led.h"
// #include "framebuffer.h"
// #include "py_time.h"

const int UART_BAUDRATE = 115200;
const uint8_t MAV_system_id = 1;
const uint8_t MAV_component_id = 0x54;
uint8_t packet_sequence = 0;

// Below 0.1 or so (YMMV) and the results are just noise.
const float MAV_OPTICAL_FLOW_confidence_threshold = 0.1f;

// LED control
LED* led = nullptr;
int led_state = 0;

void update_led() {
    led_state = led_state + 1;
    if (led_state == 5) {
        led->on();
    } else if (led_state >= 10) {
        led->off();
        led_state = 0;
    }
}

// Link Setup
UART* uart = nullptr;

// https://github.com/mavlink/c_library_v1/blob/master/checksum.h
uint16_t checksum(const uint8_t* data, size_t len, uint8_t extra) {
    uint16_t output = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        uint8_t tmp = data[i] ^ (output & 0xFF);
        tmp = (tmp ^ (tmp << 4)) & 0xFF;
        output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
    }
    uint8_t tmp = extra ^ (output & 0xFF);
    tmp = (tmp ^ (tmp << 4)) & 0xFF;
    output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF;
    return output;
}

const uint8_t MAV_OPTICAL_FLOW_message_id = 100;
const uint8_t MAV_OPTICAL_FLOW_id = 0;  // unused
const uint8_t MAV_OPTICAL_FLOW_extra_crc = 175;

// http://mavlink.org/messages/common#OPTICAL_FLOW
// https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_optical_flow.h
void send_optical_flow_packet(int x, int y, float c) {
    // Pack the 26-byte payload
    uint8_t payload[26];
    
    // <qfffhhbb format: uint64_t, float, float, float, int16_t, int16_t, int8_t, int8_t
    uint64_t time_usec = 0;
    float flow_comp_m_x = 0.0f;
    float flow_comp_m_y = 0.0f;
    float ground_distance = 0.0f;
    int16_t flow_x = static_cast<int16_t>(x);
    int16_t flow_y = static_cast<int16_t>(y);
    int8_t sensor_id = static_cast<int8_t>(MAV_OPTICAL_FLOW_id);
    int8_t quality = static_cast<int8_t>(c * 255);
    
    // Manual packing in little-endian format
    size_t offset = 0;
    memcpy(payload + offset, &time_usec, sizeof(uint64_t)); offset += sizeof(uint64_t);
    memcpy(payload + offset, &flow_comp_m_x, sizeof(float)); offset += sizeof(float);
    memcpy(payload + offset, &flow_comp_m_y, sizeof(float)); offset += sizeof(float);
    memcpy(payload + offset, &ground_distance, sizeof(float)); offset += sizeof(float);
    memcpy(payload + offset, &flow_x, sizeof(int16_t)); offset += sizeof(int16_t);
    memcpy(payload + offset, &flow_y, sizeof(int16_t)); offset += sizeof(int16_t);
    memcpy(payload + offset, &sensor_id, sizeof(int8_t)); offset += sizeof(int8_t);
    memcpy(payload + offset, &quality, sizeof(int8_t)); offset += sizeof(int8_t);
    
    // Pack the MAVLink header + payload (31 bytes)
    uint8_t header_payload[31];
    header_payload[0] = 26;  // payload length
    header_payload[1] = packet_sequence & 0xFF;
    header_payload[2] = MAV_system_id;
    header_payload[3] = MAV_component_id;
    header_payload[4] = MAV_OPTICAL_FLOW_message_id;
    memcpy(header_payload + 5, payload, 26);
    
    // Calculate checksum
    uint16_t crc = checksum(header_payload, 31, MAV_OPTICAL_FLOW_extra_crc);
    
    // Build final packet (34 bytes total)
    uint8_t packet[34];
    packet[0] = 0xFE;  // MAVLink v1 start byte
    memcpy(packet + 1, header_payload, 31);
    memcpy(packet + 32, &crc, sizeof(uint16_t));
    
    packet_sequence++;
    uart->write(packet, 34);
    update_led();
}