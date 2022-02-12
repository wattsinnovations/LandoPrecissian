#include <Mavlink.hpp>

namespace mavlink {

Mavlink::Mavlink(const std::string& local_ip, int local_port)
{
	_connection = new UdpConnection(local_ip, local_port, std::bind(&Mavlink::handle_message,
																	this, std::placeholders::_1));
	// Sets up the connection and starts the receiving thread
	_connection->start();
}

Mavlink::~Mavlink()
{
	_connection->stop();
	delete _connection;
}

void Mavlink::handle_message(const mavlink_message_t& message)
{
	// Filter messages -- we only care about
	// - MAVLINK_MSG_ID_COMMAND_LONG: from Autopilot or GCS
	// - MAVLINK_MSG_ID_DISTANCE_SENSOR: from Autopilot
	switch (message.msgid) {
		default:
			break;
	}
}

void Mavlink::send_heartbeat()
{
	mavlink_message_t message;

	mavlink_msg_heartbeat_pack(
		AUTOPILOT_SYS_ID,
		TEST_COMPONENT_ID,
		&message,
		TEST_COMPONENT_ID,
		MAV_AUTOPILOT_INVALID,
		0,
		0,
		0);

	if (_connection->connected()) {
		_connection->send_message(message);
	}
}

} // end namespace mavlink
