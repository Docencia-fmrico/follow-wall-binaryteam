#include "practica1_pkg/WallFollower.hpp"

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<WallFollower>();

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}