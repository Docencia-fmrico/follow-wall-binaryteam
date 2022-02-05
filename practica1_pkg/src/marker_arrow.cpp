#include "visualization_msgs/msg/marker.hpp" 
#include "rclcpp/rclcpp.hpp"

#include "practica1_pkg/marker_arrow.hpp"

namespace marker_arrow
{
    ArrowMarkerPublisher::ArrowMarkerPublisher(geometry_msgs::msg::Pose pose, string ns, int id)
    : Node("marker_arrow_pub_node"), counter(0)
    {
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("marker", 100);
        set_marker_values(pose, visualization_msgs::msg::Marker::ADD);

        marker_.header.frame_id = "/base_link";
        marker_.header.stamp = ros::Time::now();
        marker_.ns = ns;
        marker_.id = id;
        marker_.type = visualization_msgs::msg::Marker::ARROW;
        set_marker_values(pose, visualization_msgs::msg::Marker::ADD);
    }

    void ArrowMarkerPublisher::set_marker_values(geometry_msgs::msg::Pose pose, int action){
        marker_.action = action;
        marker_.pose = pose;
        marker_.scale.x = 1.0; 
        marker_.scale.y = 1.0; 
        marker_.scale.z = 1.0; 
        marker_.color.r = 0.0f;
        marker_.color.g = 1.0f;
        marker_.color.b = 0.0f;
        marker_.color.a = 1.0;
        marker_.lifetime = ros::Duration();
    }

    void ArrowMarkerPublisher::publish_marker(){
        marker_pub_->publish(marker_);
    }
}