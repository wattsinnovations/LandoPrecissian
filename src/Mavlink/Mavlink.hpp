#pragma once

#include <udp_connection.hpp>

#include <atomic>
#include <queue>
#include <mutex>

namespace mavlink {

static constexpr uint8_t QGROUNDCONTROL_SYS_ID = 255;
static constexpr uint8_t AUTOPILOT_SYS_ID = 1;
static constexpr uint8_t TEST_COMPONENT_ID = 70;

struct VehicleOdometry {
	float x {};
	float y {};
	float z {};
	float q[4] {};
	uint64_t last_time {};
};

class Mavlink {
public:
	Mavlink(const std::string& local_ip, int local_port);
	~Mavlink();

	//-----------------------------------------------------------------------------
	// Message senders
	void send_heartbeat();
	void send_landing_target(float angle_x, float angle_y);

	//-----------------------------------------------------------------------------
	// Helpers
	const VehicleOdometry vehicle_odometry() {
		std::lock_guard<std::mutex> lock(_odometry_mutex);
		auto data = _vehicle_odometry;
		return data;
	};
	bool connected() const { return _connection->connected(); };

private:
	//-----------------------------------------------------------------------------
	// Message handlers
	void handle_message(const mavlink_message_t& message);
	void handle_odometry(const mavlink_odometry_t& message);


	//-----------------------------------------------------------------------------
	// Message senders

private:

	UdpConnection* _connection {};

	std::mutex _odometry_mutex {};
	VehicleOdometry _vehicle_odometry {};
};

} // end namespace mavlink