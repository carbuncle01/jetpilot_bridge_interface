#include <chrono>
#include <memory>

#include "jetpilot_bridge_interface/jetpilot_bridge_interface_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "gtest/gtest.h"

namespace jetpilot_bridge_interface
{
namespace
{

using namespace std::chrono_literals;

class SteeringOffsetTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}

  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(SteeringOffsetTest, JoyOffsetTopicsAdjustTheRuntimeParameter) {
  auto bridge = std::make_shared<JetpilotBridgeInterfaceNode>();
  EXPECT_DOUBLE_EQ(bridge->get_parameter("steering_scale").as_double(), -1.0);
  auto test_node =
    std::make_shared<rclcpp::Node>("steering_offset_test_publisher");
  auto increment =
    test_node->create_publisher<std_msgs::msg::Bool>("/steer_offset_inc", 10);
  auto decrement =
    test_node->create_publisher<std_msgs::msg::Bool>("/steer_offset_dec", 10);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge);
  executor.add_node(test_node);

  const auto wait_for = [&executor](const auto & condition) {
      const auto deadline = std::chrono::steady_clock::now() + 2s;
      while (std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        if (condition()) {
          return true;
        }
      }
      return false;
    };

  ASSERT_TRUE(wait_for([&increment, &decrement]() {
      return increment->get_subscription_count() == 1U &&
             decrement->get_subscription_count() == 1U;
  }));

  std_msgs::msg::Bool message;
  message.data = true;
  increment->publish(message);
  ASSERT_TRUE(wait_for([&bridge]() {
      return bridge->get_parameter("steering_offset").as_double() == 0.01;
  }));

  decrement->publish(message);
  ASSERT_TRUE(wait_for([&bridge]() {
      return bridge->get_parameter("steering_offset").as_double() == 0.0;
  }));

  message.data = false;
  increment->publish(message);
  executor.spin_some();
  EXPECT_DOUBLE_EQ(bridge->get_parameter("steering_offset").as_double(), 0.0);
}

} // namespace
} // namespace jetpilot_bridge_interface
