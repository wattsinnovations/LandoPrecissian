// Testing out ros2!

#include <iostream>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

#include <Mavlink.hpp>

static constexpr uint64_t HEARTBEAT_INTERVAL_MS = 	500; // 1Hz
static constexpr uint64_t LANDING_TARGET_INTERVAL_MS = 	100; // 10Hz
static constexpr int PX4_TARGET_TIMEOUT_SECONDS = 	2; // 1Hz

class ArucoMarkerProcessor : public rclcpp::Node {
public:
ArucoMarkerProcessor()
		: Node("lando_precissian")
		, _last_update_time(this->get_clock()->now())
	{
		std::string version(APP_GIT_VERSION);
		RCLCPP_INFO(this->get_logger(), "Version: " GREEN_TEXT "%s" NORMAL_TEXT, version.c_str());

		auto callback = [this](ros2_aruco_interfaces::msg::ArucoMarkers::UniquePtr msg) {
			_tag_point = msg->poses[0].position;
			_tag_quat = msg->poses[0].orientation;
			_last_update_time = this->get_clock()->now();

			if (millis() < (this->_mavlink->vehicle_odometry().last_time + 1000)) {
				this->send_landing_target();
			}
		};

		_aruco_sub = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10, callback);

		// Starts the mavlink connection interface
		_mavlink = new mavlink::Mavlink("127.0.0.1", 14562);
	}

	void send_heartbeat();
	void send_landing_target();

	rclcpp::Time last_tag_update() { return _last_update_time; };

private:
	rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr _aruco_sub;

	rclcpp::Time _last_update_time;
	geometry_msgs::msg::Point _tag_point; // camera reference frame
	geometry_msgs::msg::Quaternion _tag_quat; // camera reference frame

	mavlink::Mavlink* _mavlink = nullptr;
};

void ArucoMarkerProcessor::send_heartbeat()
{
	static uint64_t last_time_ms = 0;
	uint64_t time_now = millis();

	if (time_now > last_time_ms + HEARTBEAT_INTERVAL_MS) {
		_mavlink->send_heartbeat();
		last_time_ms = time_now;
		LOG("sending heartbeat");
	}
}

void ArucoMarkerProcessor::send_landing_target()
{
	static uint64_t last_time_ms = 0;
	uint64_t time_now = millis();

	if (time_now > last_time_ms + LANDING_TARGET_INTERVAL_MS) {

		mavlink::VehicleOdometry odom = _mavlink->vehicle_odometry();

		// Calculate yaw from quaternions
		// https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
		float w = odom.q[0];
		float x = odom.q[1];
		float y = odom.q[2];
		float z = odom.q[3];
		// float yaw = atan2(2.0 * (z * w + x * y) , -1.0 + 2.0 * (w * w + x * x));
		float yaw = M_PI/2.0f;
		// float yaw = 0;

		// Correct the signs on the axes (done)
		float tag_y = -_tag_point.y;
		float tag_x = _tag_point.x;
		float tag_z = _tag_point.z;

		// https://doubleroot.in/lessons/coordinate-geometry-basics/translation-rotation-examples/
		// Rotate the camera origin into the FMU Local NED frame

		// LOG("tag_n: %f", tag_y);
		// LOG("tag_e: %f", tag_x);
		// LOG("tag_d: %f", tag_z);
		// LOG("\n");

		// Perform the rotation and translation
		// static constexpr float BOOM_LENGTH = 0; // meters
		// float camera_origin_n = BOOM_LENGTH * cos(yaw);
		// float camera_origin_e = BOOM_LENGTH * sin(yaw);
		tag_x = tag_x * cos(yaw) - tag_y * sin(yaw);
		tag_y = tag_x * sin(yaw) + tag_y * cos(yaw);

		LOG("tag_n: %f", tag_y);
		LOG("tag_e: %f", tag_x);
		LOG("tag_d: %f", tag_z);
		LOG("\n");

		// Offset from current pos from odometry
		float target_north = odom.x + tag_y;
		float target_east = odom.y + tag_x;
		float target_down = odom.z + tag_z;

		// LOG("odom_n: %f", odom.x);
		// LOG("odom_e: %f", odom.y);
		// LOG("odom_d: %f", odom.z);
		// LOG("\n");

		// LOG("yaw: %f", yaw);
		// LOG("tgt_n: %f", target_north);
		// LOG("tgt_e: %f", target_east);
		// LOG("tgt_d: %f", target_down);
		// LOG("\n");

		// LOG("n: %f", north);
		// LOG("e: %f", east);
		// LOG("d: %f", down);



		float point[3] = { target_north, target_east, target_down };


		// float quat[4] = {(float)_tag_quat.x, (float)_tag_quat.y, (float)_tag_quat.z, (float)_tag_quat.w};
		float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Zero rotation

		_mavlink->send_landing_target(point, quat);
		last_time_ms = time_now;
		LOG("sending landing_target");
	}
}

int main(int argc, char * argv[])
{
    rclcpp::Rate loop_rate(50);
	rclcpp::init(argc, argv);

	auto node = std::make_shared<ArucoMarkerProcessor>();

	while (rclcpp::ok()) {

		// node->send_heartbeat(); // Are heartbeats even necessary?
		rclcpp::spin_some(node); // Processes callbacks until idle

		// Monitor the update jitter and report timeouts
		double dt = (node->get_clock()->now() - node->last_tag_update()).seconds();
		if (dt > PX4_TARGET_TIMEOUT_SECONDS) {
			auto& clk = *node->get_clock();
			RCLCPP_INFO_THROTTLE(node->get_logger(), clk, 1000, RED_TEXT "Aruco updates spurious..." NORMAL_TEXT);

		} else {
			// We have valid aruco_marker data coming in. Convert to mavlink
			// auto& clk = *node->get_clock();
			// RCLCPP_INFO_THROTTLE(node->get_logger(), clk, 1000, "Spinning");
		}

		loop_rate.sleep();
	}

	RCLCPP_INFO(node->get_logger(), "I exit now hehehe");

	rclcpp::shutdown();
	return 0;
}
