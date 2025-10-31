
#include "uart_interface.hpp"

UartInterface::UartInterface()
{
	InitializeDefaults();
}

UartInterface::UartInterface(const char *uart_name_, int baudrate_)
{
	InitializeDefaults();
	uart_name = uart_name_;
	baudrate  = baudrate_;
}

UartInterface::~UartInterface()
{
	pthread_mutex_destroy(&lock);
	Stop();
}

void UartInterface::InitializeDefaults()
{
	// Start mutex
	int result = pthread_mutex_init(&lock, NULL);
	if ( result != 0 )
	{
		printf("\n mutex init failed\n");
		throw 1;
	}
}

void UartInterface::Start()
{
	printf("OPEN PORT\n");

	uart_fd = OpenPort(uart_name);

	// Check success
	if (uart_fd == -1)
	{
		printf("failure, could not open port.\n");
		throw EXIT_FAILURE;
	}

	bool success = SetupPort(baudrate, 8, 1, false, false);

	if (!success)
	{
		printf("failure, could not configure port.\n");
		throw EXIT_FAILURE;
	}
	if (uart_fd <= 0)
	{
		printf("Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
		throw EXIT_FAILURE;
	}

	printf("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);
	lastStatus.packet_rx_drop_count = 0;

	is_open = true;

	printf("\n");

	return;
}

int UartInterface::OpenPort(const char *port)
{
	// Open serial port
	// O_RDWR - Read and write
	// O_NOCTTY - Ignore special chars like CTRL-C
	uart_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

	// Check for Errors
	if (uart_fd == -1)
	{
		/* Could not open the port. */
		return -1;
	}

	// Finalize
	else
	{
		fcntl(uart_fd, F_SETFL, 0);
	}

	return uart_fd;
}

bool UartInterface::SetupPort(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	// Check file descriptor
	if(!isatty(uart_fd))
	{
		fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", uart_fd);
		return false;
	}

	// Read file descritor configuration
	struct termios  config;
	if(tcgetattr(uart_fd, &config) < 0)
	{
		std::cerr << "ERROR: could not read configuration of uart_fd" << uart_fd << std::endl;
		return false;
	}

	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
						INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
						 ONOCR | OFILL | OPOST);

	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif

	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;

	// One input byte is enough to return from read()
	// Inter-character timer off
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 10; // was 0

	// Get the current options for the port
	////struct termios options;
	////tcgetattr(fd, &options);

	// Apply baudrate
	if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
	{
		fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
		return false;
	}
	return true;
}

int UartInterface::ReadPort(uint8_t &cp)
{
	// Lock
	pthread_mutex_lock(&lock);

	int result = read(uart_fd, &cp, 1);

	// Unlock
	pthread_mutex_unlock(&lock);

	return result;
}

int UartInterface::ReadMessage(mavlink_message_t &message)
{
	uint8_t          cp;
	mavlink_status_t status;
	uint8_t          msgReceived = false;

	// this function locks the port during read
	int result = ReadPort(cp);


	// --------------------------------------------------------------------------
	//   PARSE MESSAGE
	// --------------------------------------------------------------------------
	if (result > 0)
	{
		// the parsing
		msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);

		// check for dropped packets
		if ( (lastStatus.packet_rx_drop_count != status.packet_rx_drop_count) && debug )
		{
			printf("ERROR: DROPPED %d PACKETS\n", status.packet_rx_drop_count);
			unsigned char v=cp;
			fprintf(stderr,"%02x ", v);
		}
		lastStatus = status;
	}
	else
	{
		fprintf(stderr, "ERROR: Could not read from fd %d\n", uart_fd);
	}

	// --------------------------------------------------------------------------
	//   DEBUGGING REPORTS
	// --------------------------------------------------------------------------
	if(msgReceived && debug)
	{
		// Report info
		printf("Received message from serial with ID #%d (sys:%d|comp:%d):\n", message.msgid, message.sysid, message.compid);

		fprintf(stderr,"Received serial data: ");
		unsigned int i;
		uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

		// check message is write length
		unsigned int messageLength = mavlink_msg_to_send_buffer(buffer, &message);

		// message length error
		if (messageLength > MAVLINK_MAX_PACKET_LEN)
		{
			fprintf(stderr, "\nFATAL ERROR: MESSAGE LENGTH IS LARGER THAN BUFFER SIZE\n");
		}

		// print out the buffer
		else
		{
			for (i=0; i<messageLength; i++)
			{
				unsigned char v=buffer[i];
				fprintf(stderr,"%02x ", v);
			}
			fprintf(stderr,"\n");
		}
	}

	// Done!
	return msgReceived;
}

int UartInterface::WritePort(char *buf, unsigned len)
{
	// Lock
	pthread_mutex_lock(&lock);

	// Write packet via serial link
	const int bytesWritten = static_cast<int>(write(uart_fd, buf, len));

	// Wait until all data has been written
	tcdrain(uart_fd);

	// Unlock
	pthread_mutex_unlock(&lock);


	return bytesWritten;
}

void UartInterface::Stop()
{
	printf("CLOSE PORT\n");

	int result = close(uart_fd);

	if ( result )
	{
		fprintf(stderr,"WARNING: Error on port close (%i)\n", result );
	}

	is_open = false;

	printf("\n");

}

void UartInterface::SendOpticalFlow(float flow_x, float flow_y, float flow_rate_x, 
								 float float_rate_y, float quality, float ground_distance)
{
    mavlink_message_t msg;
    mavlink_optical_flow_t optical_flow;
        
    // Заполнение данных оптического потока
    optical_flow.time_usec = GetTimeUsec();
    optical_flow.sensor_id = 1;
    optical_flow.flow_x = flow_x;      // пиксели/сек
    optical_flow.flow_y = flow_y;      // пиксели/сек
    optical_flow.flow_comp_m_x = 0;    // всегда 0, т.к. считаем, что камера всегда направлена вниз
    optical_flow.flow_comp_m_y = 0;    // всегда 0, т.к. считаем, что камера всегда направлена вниз
    optical_flow.quality = quality;    // 0-255 (качество, по умолчанию 255)
    optical_flow.ground_distance = ground_distance; // метры (по умолчанию < 0, т.к. не знаем высоту)
    optical_flow.flow_rate_x = 0;
    optical_flow.flow_rate_y = 0;
        
    mavlink_msg_optical_flow_encode(1, 1, &msg, &optical_flow);
    std::cout << "bytes written: " << WriteMessage(msg) << std::endl;
}

uint64_t UartInterface::GetTimeUsec() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

int UartInterface::WriteMessage(const mavlink_message_t &message)
{
	char buf[300];

	// Translate message to buffer
	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

	// Write buffer to serial port, locks port while writing
	int bytesWritten = WritePort(buf,len);

	return bytesWritten;
}