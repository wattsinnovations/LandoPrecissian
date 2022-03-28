#pragma once

#include <udp_connection.hpp>

namespace mavlink {

static constexpr uint8_t QGROUNDCONTROL_SYS_ID = 255;
static constexpr uint8_t AUTOPILOT_SYS_ID = 1;
static constexpr uint8_t LANDO_COMPONENT_ID = 70;

class Mavlink {
public:
	Mavlink(const std::string& local_ip, int local_port);
	~Mavlink();

	void send_landing_target(float angle_x, float angle_y, float q_x, float q_y, float q_z, float q_w);
	bool connected() const { return _connection->connected(); };

private:
	void handle_message(const mavlink_message_t& message);

private:
	UdpConnection* _connection {};
};

} // end namespace mavlink