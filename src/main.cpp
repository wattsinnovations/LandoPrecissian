// Testing out ros2!

#include <iostream>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

#include <Mavlink.hpp>

static constexpr uint64_t HEARTBEAT_INTERVAL_MS = 	500; // 1Hz

class ArucoMarkerProcessor : public rclcpp::Node {
public:
ArucoMarkerProcessor()
		: Node("jakes_node")
		, _last_update_time(this->get_clock()->now())
	{
		auto callback = [this](ros2_aruco_interfaces::msg::ArucoMarkers::UniquePtr msg) {
			_tag_point = msg->poses[0].position;
			_tag_quat = msg->poses[0].orientation;
			_last_update_time = this->get_clock()->now();
		};

		_aruco_sub = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10, callback);
	}

	rclcpp::Time last_tag_update() { return _last_update_time; };

private:
	rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr _aruco_sub;

	rclcpp::Time _last_update_time;
	geometry_msgs::msg::Point _tag_point; // camera reference frame
	geometry_msgs::msg::Quaternion _tag_quat; // camera reference frame
};


mavlink::Mavlink* _mavlink = nullptr;

void send_heartbeat()
{
	static uint64_t last_heartbeat_ms = 0;
	uint64_t time_now = millis();

	if (time_now > last_heartbeat_ms + HEARTBEAT_INTERVAL_MS) {
		_mavlink->send_heartbeat();
		last_heartbeat_ms = time_now;
		LOG("sending heartbeat");
	}
}

int main(int argc, char * argv[])
{
    rclcpp::Rate loop_rate(50);
	rclcpp::init(argc, argv);

	// Aruco marker processing
	auto node = std::make_shared<ArucoMarkerProcessor>();

	// Starts the mavlink connection interface
	_mavlink = new mavlink::Mavlink("127.0.0.1", 14562);

	std::string version(APP_GIT_VERSION);
	RCLCPP_INFO(node->get_logger(), "Version: %s", version.c_str());

	while (rclcpp::ok()) {

		rclcpp::spin_some(node); // Processes callbacks until idle

		send_heartbeat();

		double dt = (node->get_clock()->now() - node->last_tag_update()).seconds();
		if (dt > 3) {
			auto& clk = *node->get_clock();
			RCLCPP_INFO_THROTTLE(node->get_logger(), clk, 1000, "No aruco_markers updates");

		} else {
			// We have valid aruco_marker data coming in. Convert to mavlink
			auto& clk = *node->get_clock();
			RCLCPP_INFO_THROTTLE(node->get_logger(), clk, 1000, "Spinning");
		}

		loop_rate.sleep();
	}

	RCLCPP_INFO(node->get_logger(), "I exit now hehehe");

	rclcpp::shutdown();
	return 0;
}
