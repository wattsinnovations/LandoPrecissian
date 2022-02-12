// Testing out ros2!

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

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

int main(int argc, char * argv[])
{
    rclcpp::Rate loop_rate(50);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<ArucoMarkerProcessor>();

	std::string version(APP_GIT_VERSION);
	RCLCPP_INFO(node->get_logger(), "Version: %s", version.c_str());

	while (rclcpp::ok()) {

		rclcpp::spin_some(node); // Processes callbacks until idle

		double dt = (node->get_clock()->now() - node->last_tag_update()).seconds();
		if (dt > 3) {
			auto& clk = *node->get_clock();
			RCLCPP_INFO_THROTTLE(node->get_logger(), clk, 1000, "No aruco_markers updates");

		} else {
			// We have valid aruco_marker data coming in. Convert to mavlink
		}

		loop_rate.sleep();
	}

		RCLCPP_INFO(node->get_logger(), "I exit now hehehe");

	rclcpp::shutdown();
	return 0;
}
