#include <memory>

#include "jetpilot_bridge_interface/jetpilot_bridge_interface_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jetpilot_bridge_interface::JetpilotBridgeInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
