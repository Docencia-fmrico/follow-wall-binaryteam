#ifndef MARKER_ARROW__ARROWMARKERPUBLISHER_HPP_
#define MARKER_ARROW__ARROWMARKERPUBLISHER_HPP_

#include "visualization_msgs/msg/marker.hpp" 
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>

namespace marker_arrow
{
	class ArrowMarkerPublisher : public rclcpp::Node{
		public:
			//when create the object, this it's created in the position indicated 
			ArrowMarkerPublisher(geometry_msgs::msg::Pose pose, string ns, int id);
			//action visualization_msgs::msg::Marker::MODIFY
			void set_marker_values(geometry_msgs::msg::Pose pose, int action);		
			void publish_marker();	

	private:
		visualization_msgs::msg::Marker marker_;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;		
	};
}
#endif //MARKER_ARROW__ARROWMARKERPUBLISHER_HPP_