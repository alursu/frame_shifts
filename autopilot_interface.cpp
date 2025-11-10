#include "autopilot_interface.hpp"
#include <iostream>


uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

AutopilotInterface::AutopilotInterface()
{

}


AutopilotInterface::
AutopilotInterface(Serial_Port *port_)
{
	// initialize attributes

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	port = port_; // port management object

}

AutopilotInterface::
~AutopilotInterface()
{}


void
AutopilotInterface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit and !received_first_message)
	{
		mavlink_message_t message;
		success = port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{
			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid
			received_first_message = true;
		} // end: if read message

		// Check for receipt of all items
		received_all = this_timestamps.heartbeat && this_timestamps.sys_status;

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}


void
AutopilotInterface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK PORT
	// --------------------------------------------------------------------------

	if ( !port->is_running() ) // PORT_OPEN
	{
		fprintf(stderr,"ERROR: port not open\n");
		throw 1;
	}


	printf("START READ THREAD \n");
	// result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	// if ( result ) throw result;
	// // now we're reading messages
	// printf("\n");
	read_messages();


	printf("CHECK FOR MESSAGES\n");
	while ( !current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}
	printf("Found\n");
	// now we know autopilot is sending messages
	printf("\n");


	if ( !system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	if ( !autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}

	// time_to_exit = true;
	// pthread_join(read_tid ,NULL);


	printf("START WRITE THREAD \n");
	// result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	// if ( result ) throw result;
	// // wait for it to be started
	// while ( !writing_status )
	// 	usleep(100000); // 10Hz
	// // now we're streaming optical flow commands
	// printf("\n");
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
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the port separately
}


void
AutopilotInterface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


void
AutopilotInterface::
start_write_thread(void)
{
	if ( writing_status == true )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_optical_flow(0, 0, 0, 0);
		return;
	}

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
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


void
AutopilotInterface::
write_optical_flow(float flow_x, float flow_y, float flow_rate_x, float flow_rate_y)
{
	
	writing_status = true;

    mavlink_optical_flow_t optical_flow;
        
    // Заполнение данных оптического потока
	optical_flow.sensor_id = system_id;
    optical_flow.flow_x = flow_x;      // пиксели/сек
    optical_flow.flow_y = flow_y;      // пиксели/сек
    optical_flow.flow_comp_m_x = 0;    // всегда 0, т.к. считаем, что камера всегда направлена вниз
    optical_flow.flow_comp_m_y = 0;    // всегда 0, т.к. считаем, что камера всегда направлена вниз
    optical_flow.quality = 255;    // 0-255 (качество, по умолчанию 255)
    optical_flow.ground_distance = -1; // метры (по умолчанию < 0, т.к. не знаем высоту)
    optical_flow.flow_rate_x = flow_rate_x;
    optical_flow.flow_rate_y = flow_rate_y;
	optical_flow.time_usec = (uint32_t) (get_time_usec());

	// set optical flow target
	{
		std::lock_guard<std::mutex> lock(current_optical_flow.mutex);
		current_optical_flow.data = optical_flow;
	}

	// write a message and signal writing
	mavlink_message_t message;
	mavlink_msg_optical_flow_encode(system_id, companion_id, &message, &optical_flow);

	// do the write
	int len = port->write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send OPTICAL_FLOW \n");

	// signal end
	writing_status = false;

	return;

}


void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	AutopilotInterface *autopilot_interface = (AutopilotInterface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	AutopilotInterface *autopilot_interface = (AutopilotInterface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}