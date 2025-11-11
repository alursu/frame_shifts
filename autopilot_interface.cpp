#include "autopilot_interface.hpp"

uint64_t get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

AutopilotInterface::AutopilotInterface()
{

}


AutopilotInterface::AutopilotInterface(std::shared_ptr<UartInterface> port)
{
	// initialize attributes
	time_to_exit_   = false;  // flag to signal thread exit

	system_id_    = 0; // system id
	autopilot_id_ = 0; // autopilot component id
	companion_id_ = 0; // companion computer component id

	current_messages_.sysid_  = system_id_;
	current_messages_.compid_ = autopilot_id_;

	port_ = port; // port management object

}

AutopilotInterface::~AutopilotInterface()
{

}


void AutopilotInterface::read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all && !time_to_exit_ && !received_first_message_)
	{
		mavlink_message_t message;
		success = port_->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{
			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages_.sysid_  = message.sysid;
			current_messages_.compid_ = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages_.heartbeat_));
					current_messages_.time_stamps_.heartbeat_ = get_time_usec();
					this_timestamps.heartbeat_ = current_messages_.time_stamps_.heartbeat_;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages_.sys_status_));
					current_messages_.time_stamps_.sys_status_ = get_time_usec();
					this_timestamps.sys_status_ = current_messages_.time_stamps_.sys_status_;
					break;
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}

			} // end: switch msgid
			received_first_message_ = true;
		} // end: if read message

		// Check for receipt of all items
		received_all = this_timestamps.heartbeat_ && this_timestamps.sys_status_;

	} // end: while not received all

	return;
}


void AutopilotInterface::start()
{
	int result;

	if ( !port_->is_running() ) // PORT_OPEN
	{
		fprintf(stderr,"ERROR: port not open\n");
		throw 1;
	}

	printf("START READ THREAD \n");
	read_messages();


	printf("CHECK FOR MESSAGES\n");
	while ( !current_messages_.sysid_ )
	{
		if ( time_to_exit_ )
			return;
		usleep(500000); // check at 2Hz
	}
	printf("Found\n");
	// now we know autopilot is sending messages
	printf("\n");


	if ( !system_id_ )
	{
		system_id_ = current_messages_.sysid_;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id_ );
	}

	if ( !autopilot_id_ )
	{
		autopilot_id_ = current_messages_.compid_;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id_);
		printf("\n");
	}


	printf("START WRITE THREAD \n");
	write_optical_flow(0,0,0,0);

	// Done!
	return;

}


void
AutopilotInterface::
stop()
{

	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit_ = true;

	printf("\n");
	// still need to close the port separately
}


void
AutopilotInterface::
handle_quit( int sig )
{

	try {
		stop();
	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}


void
AutopilotInterface::
write_optical_flow(float flow_x, float flow_y, float flow_rate_x, float flow_rate_y)
{

    mavlink_optical_flow_t optical_flow;
        
    // Заполнение данных оптического потока
	optical_flow.sensor_id = system_id_;
    optical_flow.flow_x = flow_x;      // пиксели/сек
    optical_flow.flow_y = flow_y;      // пиксели/сек
    optical_flow.flow_comp_m_x = 0;    // всегда 0, т.к. считаем, что камера всегда направлена вниз
    optical_flow.flow_comp_m_y = 0;    // всегда 0, т.к. считаем, что камера всегда направлена вниз
    optical_flow.quality = 255;    // 0-255 (качество, по умолчанию 255)
    optical_flow.ground_distance = -1; // метры (по умолчанию < 0, т.к. не знаем высоту)
    optical_flow.flow_rate_x = flow_rate_x;
    optical_flow.flow_rate_y = flow_rate_y;
	optical_flow.time_usec = (uint32_t) (get_time_usec());

	// write a message and signal writing
	mavlink_message_t message;
	mavlink_msg_optical_flow_encode(system_id_, companion_id_, &message, &optical_flow);

	// do the write
	int len = port_->write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send OPTICAL_FLOW \n");

	return;

}