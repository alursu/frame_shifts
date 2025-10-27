#pragma once

#include <cstdlib>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h> // This uses POSIX Threads
#include <signal.h>

#include "include/common/mavlink.h"

class UartMAVlink {

public:

    UartMAVlink();
    UartMAVlink(const char *uart_name_, int baudrate_);
    ~UartMAVlink();

    int read_message(mavlink_message_t &message);
	int write_message(const mavlink_message_t &message);

	bool is_running(){
		return is_open;
	}
	void start();
	void stop();
    
    void SendOpticalFlow(float flow_x, float flow_y, float quality, float ground_distance);
    uint64_t GetTimeUsec();

private:
    int uart_fd;

    mavlink_status_t lastStatus;
	pthread_mutex_t  lock;

	void initialize_defaults();

	bool debug;
	const char *uart_name;
	int  baudrate;
	bool is_open;

	int  _open_port(const char* port = "/dev/ttyAMA0");
	bool _setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
	int  _read_port(uint8_t &cp);
	int _write_port(char *buf, unsigned len);

};