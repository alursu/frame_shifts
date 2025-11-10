#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_

#include "uart_interface.hpp"
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions
#include <mutex>
#include <common/mavlink.h>

// helper functions
uint64_t get_time_usec();

void* start_autopilot_interface_read_thread(void *args);
void* start_autopilot_interface_write_thread(void *args);


// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t heartbeat;
	uint64_t sys_status;

	void
	reset_timestamps()
	{
		heartbeat = 0;
		sys_status = 0;
	}

};


// Struct containing information of the MAV we are currently connected to

struct Mavlink_Messages {

	int sysid;
	int compid;

	mavlink_heartbeat_t heartbeat;
	mavlink_sys_status_t sys_status;
	Time_Stamps time_stamps;

	void
	reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}

};


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------
/*
 * This starts two threads for read and write over MAVlink. 
 */
class AutopilotInterface
{

public:

	AutopilotInterface();
	explicit AutopilotInterface(Serial_Port *port_);
	~AutopilotInterface();

	char reading_status;
	char writing_status;
	char control_status;

    int system_id;
	int autopilot_id;
	int companion_id;

	Mavlink_Messages current_messages;

	void read_messages();
	void write_optical_flow(float flow_x, float flow_y, float flow_rate_x, float flow_rate_y);

	void start();
	void stop();

	void start_read_thread();
	void start_write_thread(void);

	void handle_quit( int sig );


private:

	Serial_Port *port;

	bool time_to_exit;

	pthread_t read_tid;
	pthread_t write_tid;

		struct {
		std::mutex mutex;
		mavlink_optical_flow_t data;
	} current_optical_flow;

	void read_thread();
	bool received_first_message = false;

};

#endif // AUTOPILOT_INTERFACE_H_