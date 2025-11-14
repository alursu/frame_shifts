#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_

#include "uart_interface.hpp"
#include <sys/time.h>
#include <common/mavlink.h>
#include <memory>
#include <iostream>

uint64_t get_time_usec();

// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t heartbeat_;
	uint64_t sys_status_;

	void
	reset_timestamps()
	{
		heartbeat_ = 0;
		sys_status_ = 0;
	}
};


// Структура, содержащая информацию о MAV, к которому мы в данный момент подключены
struct Mavlink_Messages {
	int sysid_;
	int compid_;

	mavlink_heartbeat_t heartbeat_;
	mavlink_sys_status_t sys_status_;
	Time_Stamps time_stamps_;

	void
	reset_timestamps()
	{
		time_stamps_.reset_timestamps();
	}
};


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

class AutopilotInterface
{
public:

	AutopilotInterface();
	explicit AutopilotInterface(std::shared_ptr<UartInterface> port_);
	~AutopilotInterface();

    int system_id_;
	int autopilot_id_;
	int companion_id_;

	Mavlink_Messages current_messages_;

	void read_messages();
	void write_optical_flow(bool is_flow_reset = false, float flow_x = 0, float flow_y = 0, float flow_rate_x = 0, 
							float flow_rate_y = 0, int quality = 255, float ground_distance = -1);

	void start();
	void stop();

	void handle_quit( int sig );

private:

	std::shared_ptr<UartInterface> port_;

	bool time_to_exit_;
	bool received_first_message_ = false;
};

#endif // AUTOPILOT_INTERFACE_H_