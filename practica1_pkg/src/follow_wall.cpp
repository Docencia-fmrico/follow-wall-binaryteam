#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp" 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

class Follower : public rclcpp_lifecycle::LifecycleNode {

public:
  
  Follower() : rclcpp_lifecycle::LifecycleNode("follower_node") {
   
    velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("nav_vel", 100);
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("marker", 100);
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan_raw", 100, std::bind(&Follower::laser_callback, this, _1));
  }        

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
      
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    
    velocity_pub_->on_activate();
    activated_ = true;
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
    
    velocity_pub_->on_deactivate();
    activated_ = false;
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
    
    velocity_pub_.reset();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    
    velocity_pub_.reset();
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_error(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }
  
  void do_work() 
  {
    if (activated_ && trajectory_.size() == 2) { 
      // Follow trajectory
      geometry_msgs::msg::Twist msg;

      float magnitude = trajectory_[0];
      float angle = trajectory_[1];

      msg.angular.z = -angle * 0.2; 

      if (abs(angle) < 0.4) {

        msg.linear.x = magnitude/1000;

      }
     
      velocity_pub_->publish(msg);
    }
  }

  private:

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
      // msg->ranges 665elem, grados de 110 a -110 
      RCLCPP_INFO(get_logger(), "Angulo 0 tiene una distancia de %f", msg->ranges[332]);

      std::vector<float> distances(std::begin(msg->ranges), std::end(msg->ranges)); // Convert ranges to a vector. Now we dont have to know hw many element have. More reutilizable

      std::vector< std::vector<float> > attraction(distances.size(), std::vector<float> (2, 0.0));
      std::vector< std::vector<float> > repulsion(distances.size(), std::vector<float> (2, 0.0));

      // SERA GLOBAL
      float distance_to_wall = 1;

      float result_x = 0.0;
      float result_y = 0.0;

      // For each distance
      int index = 0;

      for (float distance: distances) {

        float force = 0.0;    
        
        float angle = msg->angle_max - msg->angle_increment * index;

        if (distance < distance_to_wall) {
          // Repulse
          angle += M_PI; // Opposite direction
          force = 0;
    
        } else {
          // Attract
          force = 0;
        }
        
        result_x += std::cos(angle)*force;
        result_y += std::sin(angle)*force;

        index ++;  
      }
      // Obtain final polar vector falta la fuerza guía

      float polar_magnitude = std::sqrt(std::pow(result_x, 2) + std::pow(result_y, 2));
      float polar_angle = std::atan2(result_x, result_y);

      trajectory_ = std::vector<float>{polar_magnitude, polar_angle};
    }

  
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_; // Intelligent pointer to a velocity publisher.
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    bool activated_ = false;
    std::vector<float> trajectory_;
};

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<Follower>();

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->do_work();

    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}