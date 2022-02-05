#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "practica1_pkg/marker_arrow.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

enum Stages {
  POINT_TO_WALL = 0
};

class Follower : public rclcpp_lifecycle::LifecycleNode {

public:
  
  Follower() : rclcpp_lifecycle::LifecycleNode("follower_node") {
   
    velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("nav_vel", 100);
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
  
  void do_work() {
   
    if (activated_) {

      if (stage_ == POINT_TO_WALL) {

        int min_element_index = std::min_element(distances_.begin(), distances_.end()) - distances_.begin();
          
        float align_wall_angle = angle_max_ - angle_increment * min_element_index;
        
        geometry_msgs::msg::Twist msg;

        msg.linear.x = 0.0;

        msg.angular.z = -align_wall_angle / 8;

        velocity_pub_->publish(msg);

      }      

    }
  }

  private:

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
      // msg->ranges 665elem, grados de 110 a -110 
      RCLCPP_INFO(get_logger(), "Angulo 0 tiene una distancia de %f", msg->ranges[332]);

      distances_ = std::vector<float> (std::begin(msg->ranges), std::end(msg->ranges)); // Convert ranges to a vector. Now we dont have to know hw many element have. More reutilizable
      angle_max_ = msg->angle_max;
      angle_increment = msg->angle_increment;

    }
     
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_; // Intelligent pointer to a velocity publisher.
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    bool activated_ = false;
    int stage_ = POINT_TO_WALL;
    std::vector<float> distances_;
    float angle_max_;
    float angle_increment;
    
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