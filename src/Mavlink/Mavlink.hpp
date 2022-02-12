#pragma once

#include <udp_connection.hpp>

#include <atomic>
#include <queue>
#include <mutex>

namespace mavlink {

static constexpr uint8_t QGROUNDCONTROL_SYS_ID = 255;
static constexpr uint8_t AUTOPILOT_SYS_ID = 1;
static constexpr uint8_t TEST_COMPONENT_ID = 70;


class Mavlink {
public:
	Mavlink(const std::string& local_ip, int local_port);
	~Mavlink();

	//-----------------------------------------------------------------------------
	// Message senders
	void send_heartbeat();
	void send_landing_target(float p[3], float q[4]);

	//-----------------------------------------------------------------------------
	// Helpers

	bool connected() const { return _connection->connected(); };


private:
	//-----------------------------------------------------------------------------
	// Message handlers
	void handle_message(const mavlink_message_t& message);

	//-----------------------------------------------------------------------------
	// Message senders

private:

	UdpConnection* _connection {};


	std::mutex _command_queue_mutex {};
	std::queue<mavlink_command_long_t> _command_queue {};
};

} // end namespace mavlink