// Testing out ros2!

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"

class JakesNode : public rclcpp::Node {
public:
JakesNode()
	: Node("jakes_node")
{
	auto callback = [this](ros2_aruco_interfaces::msg::ArucoMarkers::UniquePtr msg) {
		RCLCPP_INFO(this->get_logger(), "x: %f", msg->poses[0].position.x);
		RCLCPP_INFO(this->get_logger(), "y: %f", msg->poses[0].position.y);
		RCLCPP_INFO(this->get_logger(), "z: %f", msg->poses[0].position.z);
	};

	_aruco_sub = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10, callback);
}

private:
	rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr _aruco_sub;
};

int main(int argc, char * argv[])
{
    rclcpp::Rate loop_rate(50);

	rclcpp::init(argc, argv);
	// rclcpp::spin(std::make_shared<JakesNode>()); // Processes callbacks continuously
	auto node = std::make_shared<JakesNode>();

	while (rclcpp::ok()) {

		rclcpp::spin_some(node); // Processes callbacks until idle
		// RCLCPP_INFO(node->get_logger(), "I spun");

		// Do something?

		loop_rate.sleep();
	}

		RCLCPP_INFO(node->get_logger(), "I exit now hehehe");

	rclcpp::shutdown();
	return 0;
}
