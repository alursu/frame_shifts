#include "uart_interface.hpp"


UartInterface::UartInterface(const char *uart_name , int baudrate)
{
	initialize_defaults();
	uart_name_ = uart_name;
	baudrate_  = baudrate;
}

UartInterface::UartInterface()
{
	initialize_defaults();
}

UartInterface::~UartInterface()
{

}

void UartInterface::initialize_defaults()
{
	uart_fd_     = -1;
	is_open_ = false;

	uart_name_ = (char*)"/dev/ttyUSB0";
	baudrate_  = 57600;
}



int UartInterface::read_message(mavlink_message_t &message)
{
	uint8_t          cp;
	mavlink_status_t status;
	uint8_t          msgReceived = false;

	int result = read_port(cp);
	if (result > 0)
	{
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
	}
	else
	{
		std::cerr << "ERROR: Could not read from fd " << uart_fd_ << std::endl;
	}

	return msgReceived;
}


int UartInterface::write_message(const mavlink_message_t &message)
{
	char buf[300];

	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
	int bytesWritten = write_port(buf,len);

	return bytesWritten;
}



void UartInterface::start()
{
	std::clog << "OPEN PORT" << std::endl;

	uart_fd_ = open_port(uart_name_);
	if (uart_fd_ == -1)
	{
		std::cerr << "failure, could not open port." << std::endl;
		throw EXIT_FAILURE;
	}

	bool success = setup_port(baudrate_, 8, 1, false, false);
	if (!success)
	{
		std::cerr << "failure, could not configure port." << std::endl;
		throw EXIT_FAILURE;
	}
	if (uart_fd_ <= 0)
	{
		std::cerr << "Connection attempt to port " << uart_name_ << " with " << baudrate_ 
				  << " baud, 8N1 failed, exiting." << std::endl;
		throw EXIT_FAILURE;
	}

	std::clog << "Connected to " << uart_name_ << " with " << baudrate_ 
			  << " baud, 8 data bits, no parity, 1 stop bit (8N1)" << std::endl;
	is_open_ = true;
	std::clog << std::endl;

	return;

}


void UartInterface::stop()
{
	std::clog << "CLOSE PORT" << std::endl;

	int result = close(uart_fd_);
	if ( result )
	{
		std::cerr << "WARNING: Error on port close " << result << std::endl;
	}
	is_open_ = false;
	
	std::clog << std::endl;
}



int UartInterface::open_port(const char* port)
{
	uart_fd_ = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (uart_fd_ == -1)
	{
		return -1;
	}
	else
	{
		fcntl(uart_fd_, F_SETFL, 0);
	}
	return uart_fd_;
}


bool UartInterface::setup_port(int baud, int data_bits, int stop_bits, 
							   bool parity, bool hardware_control)
{
	if(!isatty(uart_fd_))
	{
		std::cerr << std::endl << "ERROR: file descriptor " << uart_fd_ 
				  << " is NOT a serial port" << std::endl;
		return false;
	}

	struct termios  config;
	if(tcgetattr(uart_fd_, &config) < 0)
	{
		std::cerr << std::endl << "ERROR: could not read configuration of fd " 
				  << uart_fd_ << std::endl;
		return false;
	}

	// Флаги ввода
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
						INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Флаги вывода
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
						 ONOCR | OFILL | OPOST);

	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10;

	if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
		{
			std::cerr << std::endl << "ERROR: Could not set desired baud rate of "
					  << baud << " Baud" << std::endl;
			return false;
		}

	if(tcsetattr(uart_fd_, TCSAFLUSH, &config) < 0)
	{
		std::cerr << std::endl << "ERROR: could not set configuration of fd "
				  << uart_fd_ << std::endl;
		return false;
	}

	return true;
}



int UartInterface::read_port(uint8_t &cp)
{
	int result = read(uart_fd_, &cp, 1);
	return result;
}


int UartInterface::write_port(char *buf, unsigned len)
{
	// Очищаем буфер и отправляем данные
	tcflush(uart_fd_, TCIFLUSH); 
	const int bytesWritten = static_cast<int>(write(uart_fd_, buf, len));

	// Ждем, пока все данные отправятся
	tcdrain(uart_fd_);
	return bytesWritten;
}