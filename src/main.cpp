// Testing out ros2!

#include <iostream>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

#include <Mavlink.hpp>

class ArucoMarkerProcessor : public rclcpp::Node {
public:
	ArucoMarkerProcessor();
	void send_landing_target(float x, float y, float z);
	bool mavlink_connected() { return _mavlink->connected(); };
	rclcpp::Time last_tag_update() { return _last_update_time; };

private:
	rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr _aruco_sub;

	rclcpp::Time _last_update_time;
	mavlink::Mavlink* _mavlink = nullptr;
};

ArucoMarkerProcessor::ArucoMarkerProcessor()
	: Node("lando")
	, _last_update_time(this->get_clock()->now())
{
	std::string version(APP_GIT_VERSION);
	RCLCPP_INFO(this->get_logger(), "Version: " GREEN_TEXT "%s" NORMAL_TEXT, version.c_str());

	auto callback = [this](ros2_aruco_interfaces::msg::ArucoMarkers::UniquePtr msg) {
		_last_update_time = this->get_clock()->now();
		this->send_landing_target(msg->poses[0].position.x, msg->poses[0].position.y, msg->poses[0].position.z);
	};

	_aruco_sub = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10, callback);

	// Starts the mavlink connection interface
	// std::string ip = "127.0.0.1";
	std::string ip = "10.223.0.70"; // IP address of Skynode AI
	int port = 14562;
	_mavlink = new mavlink::Mavlink(ip, port);
	RCLCPP_INFO(this->get_logger(), "IP: %s Port: %d", ip.c_str(), port);
}

// We use the PX4 gazebo hack implemenation to fake an irlock so that we can reuse the landing_target_estimator
//
// PX4 Implementation
// 			irlock_report.timestamp = hrt_absolute_time();
// UNUSED: 	irlock_report.signature = landing_target.target_num;
// 			irlock_report.pos_x = landing_target.angle_x;
// 			irlock_report.pos_y = landing_target.angle_y;
// UNUSED: 	irlock_report.size_x = landing_target.size_x;
// UNUSED: 	irlock_report.size_y = landing_target.size_y;
//
// When looking along the optical axis of the camera, x points right, y points down, and z points along the optical axis.
// float32 pos_x # tan(theta), where theta is the angle between the target and the camera center of projection in camera x-axis
// float32 pos_y # tan(theta), where theta is the angle between the target and the camera center of projection in camera y-axis

// https://mavlink.io/en/services/landing_target.html#positional
// https://github.com/PX4/PX4-Autopilot/pull/14959
void ArucoMarkerProcessor::send_landing_target(float x, float y, float z)
{
	// Convert to unit vector
	float r = sqrtf(x*x + y*y + z*z);
	x = x/r;
	y = y/r;
	z = z/r;
	// https://github.com/PX4/PX4-SITL_gazebo/blob/master/src/gazebo_irlock_plugin.cpp
	float tan_theta_x = x / z;
	float tan_theta_y = y / z;

	_mavlink->send_landing_target(tan_theta_x, tan_theta_y);
}

int main(int argc, char * argv[])
{
    rclcpp::Rate loop_rate(50);
	rclcpp::init(argc, argv);

	auto node = std::make_shared<ArucoMarkerProcessor>();

	while (rclcpp::ok()) {

		rclcpp::spin_some(node); // Processes callbacks until idle

		// Monitor the update jitter and report timeouts
		double dt = (node->get_clock()->now() - node->last_tag_update()).seconds();

		static int state = 0;
		static constexpr int PX4_TARGET_TIMEOUT_SECONDS = 1;

		if (dt > PX4_TARGET_TIMEOUT_SECONDS) {
			if (state != 1) {
				RCLCPP_INFO(node->get_logger(), RED_TEXT "No markers detected" NORMAL_TEXT);
				state = 1;
			}

		} else {
			if (state != 0) {
				RCLCPP_INFO(node->get_logger(), GREEN_TEXT "Processing markers: " NORMAL_TEXT "mavlink ? %u", node->mavlink_connected());
				state = 0;
			}
		}

		loop_rate.sleep();
	}

	RCLCPP_INFO(node->get_logger(), "Exiting");

	rclcpp::shutdown();
	return 0;
}
