
#include "uartmavlink.hpp"

bool UartMAVLink::UartInit(const char* device, int baudrate) {
    uart_fd = open(device, O_RDWR | O_NOCTTY);
    if (uart_fd < 0) return false;
        
    struct termios options;
    tcgetattr(uart_fd, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
        
    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
    options.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
        
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;
        
    return tcsetattr(uart_fd, TCSANOW, &options) == 0;
}
    
void UartMAVLink::SendMavlinkMessage(mavlink_message_t& msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buffer, &msg);
    write(uart_fd, buffer, len);
}
    
void UartMAVLink::SendOpticalFlow(float flow_x, float flow_y, 
                       float quality, float ground_distance) {
    mavlink_message_t msg;
    mavlink_optical_flow_t optical_flow;
        
    // Заполнение данных оптического потока
    optical_flow.time_usec = GetTimeUsec();
    optical_flow.sensor_id = 1;
    optical_flow.flow_x = flow_x;      // пиксели/сек
    optical_flow.flow_y = flow_y;      // пиксели/сек
    optical_flow.flow_comp_m_x = 0;
    optical_flow.flow_comp_m_y = 0;
    optical_flow.quality = quality;    // 0-255 (качество)
    optical_flow.ground_distance = ground_distance; // метры
        
    mavlink_msg_optical_flow_encode(1, 1, &msg, &optical_flow);
    SendMavlinkMessage(msg);
}

void UartMAVLink::SendVisionPositionDelta(float delta_x, float delta_y, float delta_z, float delta_roll, float delta_pitch, float delta_yaw)
{
    mavlink_message_t msg;
    mavlink_vision_position_delta_t vision_delta;
        
    vision_delta.time_usec = GetTimeUsec();
    vision_delta.time_delta_usec = 10000; // 10ms
    vision_delta.angle_delta[0] = delta_roll;
    vision_delta.angle_delta[1] = delta_pitch;
    vision_delta.angle_delta[2] = delta_yaw;
    vision_delta.position_delta[0] = delta_x;
    vision_delta.position_delta[1] = delta_y;
    vision_delta.position_delta[2] = delta_z;
    vision_delta.confidence = 0.8f;
        
    mavlink_msg_vision_position_delta_encode(1, 1, &msg, &vision_delta);
    SendMavlinkMessage(msg);
}
    
uint64_t UartMAVLink::GetTimeUsec() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}