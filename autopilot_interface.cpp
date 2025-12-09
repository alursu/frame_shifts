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
	time_to_exit_   = false;  // Флаг окончания работы программы

	system_id_    = 0; // system id
	autopilot_id_ = 0; // autopilot component id
	companion_id_ = 0; // companion computer component id

	current_messages_.sysid_  = system_id_;
	current_messages_.compid_ = autopilot_id_;

	port_ = port; // Объект управления портом ввода-вывода
}

AutopilotInterface::~AutopilotInterface()
{

}


void AutopilotInterface::read_messages()
{
	bool success;               // Флаг успешного получения данных
	bool received_all = false;  // Получаем только 1 сообщение
	Time_Stamps this_timestamps;

	while ( !received_all && !time_to_exit_ && !received_first_message_)
	{
		mavlink_message_t message;
		success = port_->read_message(message);

		if( success )
		{
			// Сохраняем sysid и compid сообщения.
			// Обрабатываем только один источник сообщений.
			current_messages_.sysid_  = message.sysid;
			current_messages_.compid_ = message.compid;

			// Обработка Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					mavlink_msg_heartbeat_decode(&message, &(current_messages_.heartbeat_));
					current_messages_.time_stamps_.heartbeat_ = get_time_usec();
					this_timestamps.heartbeat_ = current_messages_.time_stamps_.heartbeat_;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					mavlink_msg_sys_status_decode(&message, &(current_messages_.sys_status_));
					current_messages_.time_stamps_.sys_status_ = get_time_usec();
					this_timestamps.sys_status_ = current_messages_.time_stamps_.sys_status_;
					break;
				}

				default:
				{
					break;
				}

			} 
			received_first_message_ = true;
		}

		// Проверка получения всех данных
		received_all = this_timestamps.heartbeat_ && this_timestamps.sys_status_;

	} 

	return;
}


void AutopilotInterface::start()
{
	int result;

	if ( !port_->is_running() ) // Порт открыт
	{
		std::cerr << "ERROR: port not open" << std::endl;
		throw 1;
	}

	std::clog << "START READ THREAD" << std::endl;
	read_messages();

	std::clog << "CHECK FOR MESSAGES" << std::endl;
	while ( !current_messages_.sysid_ )
	{
		if ( time_to_exit_ )
			return;
		usleep(500000); // 2 Гц
	}
	std::clog << "Found" << std::endl;
	std::clog << std::endl;

	if ( !system_id_ )
	{
		system_id_ = current_messages_.sysid_;
		std::clog << "GOT VEHICLE SYSTEM ID: " << system_id_ << std::endl;
	}

	if ( !autopilot_id_ )
	{
		autopilot_id_ = current_messages_.compid_;
		std::clog << "GOT AUTOPILOT COMPONENT ID: " << autopilot_id_ << std::endl;
		std::clog << std::endl;
	}

	std::clog << "START WRITE THREAD" << std::endl;
	write_optical_flow();

	return;
}


void AutopilotInterface::stop()
{
	std::clog << "CLOSE THREADS" << std::endl;

	// Сигнал окончания работы программы
	time_to_exit_ = true;

	std::clog << std::endl;
}


void AutopilotInterface::handle_quit( int sig )
{
	write_optical_flow(true);
	try {
		stop();
	}
	catch (int error) {
		std::cerr << "Warning, could not stop autopilot interface" << std::endl;
	}
}


void AutopilotInterface::write_optical_flow(bool is_flow_reset, float flow_x, float flow_y, float flow_rate_x, 
											float flow_rate_y, int quality, float ground_distance)
{
    mavlink_optical_flow_t optical_flow;

	if (is_flow_reset){
		// Сброс данных оптического потока
		optical_flow.sensor_id = system_id_;
		optical_flow.flow_x = 0;  
		optical_flow.flow_y = 0;  
		optical_flow.flow_comp_m_x = 0;  
		optical_flow.flow_comp_m_y = 0;  
		optical_flow.quality = 0;    
		optical_flow.ground_distance = 0; 
		optical_flow.flow_rate_x = 0;
		optical_flow.flow_rate_y = 0;
		optical_flow.time_usec = 0;
	} else {
		// Заполнение данных оптического потока
		optical_flow.sensor_id = system_id_;
		optical_flow.flow_x = flow_x;      // пиксели/сек
		optical_flow.flow_y = flow_y;      // пиксели/сек
		optical_flow.flow_comp_m_x = 0;    // всегда 0, т.к. считаем, что камера всегда направлена вниз
		optical_flow.flow_comp_m_y = 0;    // всегда 0, т.к. считаем, что камера всегда направлена вниз
		optical_flow.quality = quality;    // 0-255 (качество, по умолчанию 255)
		optical_flow.ground_distance = ground_distance; // метры (по умолчанию < 0, т.к. не знаем высоту)
		optical_flow.flow_rate_x = flow_rate_x;
		optical_flow.flow_rate_y = flow_rate_y;
		optical_flow.time_usec = (uint32_t) (get_time_usec());
	}

	mavlink_message_t message;
	mavlink_msg_optical_flow_encode(system_id_, companion_id_, &message, &optical_flow);

	int len = port_->write_message(message);

	// Проверка отправки
	if ( len <= 0 )
		std::cerr << "WARNING: could not send OPTICAL_FLOW" << std::endl;

	return;
}