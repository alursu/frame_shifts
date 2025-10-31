#ifndef UART_INTERFACE_H_
#define UART_INTERFACE_H_

#include <cstdlib>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h> 
#include <signal.h>

#include <common/mavlink.h>

class UartInterface {

public:

    UartInterface();
    UartInterface(const char *uart_name_, int baudrate_);
    ~UartInterface();

	bool IsRunning(){
		return is_open;
	}
	void Start();
	void Stop();
	void SendOpticalFlow(float flow_x, float flow_y, float flow_rate_x, 
						float float_rate_y, float quality = 255, float ground_distance = -1);

	int ReadMessage(mavlink_message_t &message);
	int WriteMessage(const mavlink_message_t &message);

private:
    int uart_fd = -1;

    mavlink_status_t lastStatus;
	pthread_mutex_t  lock;

	void InitializeDefaults();

	bool debug = false;
	const char *uart_name = (char*)"/dev/ttyAMA0";
	int  baudrate = 115200;
	bool is_open = false;

	int  OpenPort(const char* port = "/dev/ttyAMA0");
	bool SetupPort(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);

	int  ReadPort(uint8_t &cp);
	int  WritePort(char *buf, unsigned len);

	uint64_t GetTimeUsec();

};

#endif // UART_INTERFACE_H_