#include <Mavlink.hpp>

namespace mavlink {

Mavlink::Mavlink(const std::string& local_ip, int local_port)
{
	_connection = new UdpConnection(local_ip, local_port, std::bind(&Mavlink::handle_message, this, std::placeholders::_1));
	_connection->start();
}

Mavlink::~Mavlink()
{
	_connection->stop();
	delete _connection;
}

void Mavlink::handle_message(const mavlink_message_t& message)
{
	switch (message.msgid) {
		default:
			break;
	}
}

void Mavlink::send_landing_target(float angle_x, float angle_y, float q_x, float q_y, float q_z, float q_w)
{
	mavlink_message_t message;
	mavlink_landing_target_t landing_target = {};

	// X-axis / Y-axis angular offset of the target from the center of the image
	landing_target.angle_x = angle_x;
	landing_target.angle_y = angle_y;

	// Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	landing_target.q[0] = q_w;
	landing_target.q[1] = q_x;
	landing_target.q[2] = q_y;
	landing_target.q[3] = q_z;


	mavlink_msg_landing_target_encode(AUTOPILOT_SYS_ID, LANDO_COMPONENT_ID, &message, &landing_target);

	if (_connection->connected()) {
		_connection->send_message(message);
	}
}

} // end namespace mavlink
