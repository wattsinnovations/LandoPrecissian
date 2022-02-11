// Testing out ros2!

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class JakesNode : public rclcpp::Node {
public:
JakesNode()
	: Node("jakes_node")
{
	auto callback = [this](std_msgs::msg::String::UniquePtr msg) {
		RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
	};

	_subscription = this->create_subscription<std_msgs::msg::String>("topic", 10, callback);
}

private:
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscription;
};

int main(int argc, char * argv[])
{
    rclcpp::Rate loop_rate(1);

	rclcpp::init(argc, argv);
	// rclcpp::spin(std::make_shared<JakesNode>()); // Processes callbacks continuously
	auto node = std::make_shared<JakesNode>();

	while (rclcpp::ok()) {

		rclcpp::spin_some(node); // Processes callbacks until idle
		RCLCPP_INFO(node->get_logger(), "I spun");

		// Do something?

		loop_rate.sleep();
	}

		RCLCPP_INFO(node->get_logger(), "I exit now hhehehe");

	rclcpp::shutdown();
	return 0;
}
