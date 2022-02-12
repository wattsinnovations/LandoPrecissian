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

void Mavlink::send_landing_target(float p[3], float q[4])
{
	mavlink_message_t message;

	uint64_t time_usec = micros();

	// UNUSED???
	float angle_x = {};
	float angle_y = {};
	float distance = {};
	float size_x = {};
	float size_y = {};
	uint8_t target_num = {};

	// https://mavlink.io/en/services/landing_target.html#positional
	uint8_t frame = MAV_FRAME_LOCAL_NED;
	uint8_t type = LANDING_TARGET_TYPE_VISION_FIDUCIAL;
	uint8_t position_valid = true;

	mavlink_msg_landing_target_pack(
		AUTOPILOT_SYS_ID,
		TEST_COMPONENT_ID,
		&message,
		time_usec,
		angle_x,
		angle_y,
		distance,
		size_x,
		size_y,
		target_num,
		frame,
		p[0],
		p[1],
		p[2],
		q,
		type,
		position_valid);

	if (_connection->connected()) {
		_connection->send_message(message);
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
