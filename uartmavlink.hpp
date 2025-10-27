#pragma once

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include "include/common/mavlink.h"

class UartMAVLink {
private:
    int uart_fd;
    
    bool UartInit(const char* device, int baudrate);
    void SendMavlinkMessage(mavlink_message_t& msg); 
    
public:
    bool Begin(const char* uart_port = "/dev/ttyAMA0", int baudrate = B115200) {
        return UartInit(uart_port, baudrate);
    }
    
    void SendOpticalFlow(float flow_x, float flow_y, float quality, float ground_distance);
    void SendVisionPositionDelta(float delta_x, float delta_y, float delta_z,
                                    float delta_roll, float delta_pitch, float delta_yaw); 
    uint64_t GetTimeUsec();
};