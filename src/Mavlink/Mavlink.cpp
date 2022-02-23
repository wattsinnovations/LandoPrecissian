#include <Mavlink.hpp>

namespace mavlink {

Mavlink::Mavlink(const std::string& local_ip, int local_port)
{
	_connection = new UdpConnection(local_ip, local_port, std::bind(&Mavlink::handle_message, this, std::placeholders::_1));
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
		case MAVLINK_MSG_ID_ODOMETRY:
		{
			// mavlink_odometry_t msg;
			// mavlink_msg_odometry_decode(&message, &msg);
			// handle_odometry(msg);
			break;
		}
		default:
			break;
	}
}

void Mavlink::send_landing_target(float angle_x, float angle_y)
{
	mavlink_message_t message;

	mavlink_landing_target_t landing_target = {};

	landing_target.angle_x = angle_x;
	landing_target.angle_y = angle_y;

	mavlink_msg_landing_target_encode(AUTOPILOT_SYS_ID, TEST_COMPONENT_ID, &message, &landing_target);

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

void Mavlink::handle_odometry(const mavlink_odometry_t& message)
{
	// LOG("handle_odometry");
	std::lock_guard<std::mutex> lock(_odometry_mutex);
	_vehicle_odometry.x = message.x;
	_vehicle_odometry.y = message.y;
	_vehicle_odometry.z = message.z;
	_vehicle_odometry.q[0] = message.q[0];
	_vehicle_odometry.q[1] = message.q[1];
	_vehicle_odometry.q[2] = message.q[2];
	_vehicle_odometry.q[3] = message.q[3];
	_vehicle_odometry.last_time = millis();
}

} // end namespace mavlink
