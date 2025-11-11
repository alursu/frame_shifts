#ifndef UART_INTERFACE_H_
#define UART_INTERFACE_H_


#include <stdio.h>
#include <fcntl.h>   
#include <termios.h> 
#include <signal.h>

#include <common/mavlink.h>

// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------

class UartInterface
{
public:

	UartInterface();
	UartInterface(const char *uart_name, int baudrate);
	virtual ~UartInterface();

	int read_message(mavlink_message_t &message);
	int write_message(const mavlink_message_t &message);

	bool is_running(){
		return is_open_;
	}
	void start();
	void stop();

private:

	int  uart_fd_;

	void initialize_defaults();

	const char *uart_name_;
	int  baudrate_;
	bool is_open_;

	int  open_port(const char* port);
	bool setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);
	int  read_port(uint8_t &cp);
	int  write_port(char *buf, unsigned len);
};

#endif // UART_INTERFACE_H_