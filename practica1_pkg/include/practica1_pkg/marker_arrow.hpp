#ifndef MARKER_ARROW__ARROWMARKERPUBLISHER_HPP_
#define MARKER_ARROW__ARROWMARKERPUBLISHER_HPP_

#include "visualization_msgs/msg/marker.hpp" 
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ArrowMarkerPublisher : public rclcpp::Node{
	public:
		ArrowMarkerPublisher()
		: Node("marker_arrow_pub_node"), counter(0)
		{
			marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("marker", 100);
		}

		void doWork()
		{
				std_msgs::msg::String message;
				message.data = "Hello, world! " + std::to_string(counter++);

				RCLCPP_INFO(get_logger(), "Publishing [%s]", message.data.c_str());

				pub_->publish(message);
		}

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  int counter;
};
}
#endif //MARKER_ARROW__ARROWMARKERPUBLISHER_HPP_