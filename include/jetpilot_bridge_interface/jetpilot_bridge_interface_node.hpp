#ifndef JETPILOT_BRIDGE_INTERFACE__JETPILOT_BRIDGE_INTERFACE_NODE_HPP_
#define JETPILOT_BRIDGE_INTERFACE__JETPILOT_BRIDGE_INTERFACE_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "jetpilot_bridge_interface/bridge_protocol.hpp"
#include "jetpilot_msgs/msg/control_command.hpp"
#include "jetpilot_msgs/msg/operation_mode_request.hpp"
#include "jetpilot_msgs/msg/operation_mode_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace jetpilot_bridge_interface
{

class SerialPort;

class JetpilotBridgeInterfaceNode : public rclcpp::Node
{
public:
  JetpilotBridgeInterfaceNode();
  ~JetpilotBridgeInterfaceNode() override;
  using SteadyClock = std::chrono::steady_clock;
  using SteadyTime = SteadyClock::time_point;

  enum class HostArmState : std::uint8_t
  {
    disarmed = 0,
    arming_neutral,
    armed,
  };

private:
  void update();
  void ensure_serial_open();
  void handle_serial_disconnect(const std::string & reason);
  bool command_is_fresh() const;
  bool status_is_fresh() const;
  static bool is_host_mode(std::uint8_t mode);
  void write_command();
  void read_status();
  void publish_status(const StatusFrame & status);
  void publish_mode_request_if_needed(const StatusFrame & status);
  void publish_mode_request(std::uint8_t mode, const std::string & source);
  void publish_diagnostics_if_due();
  void shift_steering_offset(double direction);

  std::string device_;
  int baud_rate_{115200};
  double command_rate_hz_{100.0};
  double command_timeout_s_{0.2};
  double status_timeout_s_{0.5};
  double reconnect_interval_s_{1.0};
  bool require_status_for_auto_{true};
  bool publish_mode_request_{true};
  std::string frame_id_;
  std::string hardware_id_;

  std::unique_ptr<SerialPort> serial_;
  std::optional<SteadyTime> serial_open_time_;
  std::chrono::steady_clock::time_point next_reconnect_attempt_{};
  std::uint32_t sequence_{0};
  std::uint64_t protocol_errors_{0};
  std::uint64_t write_drops_{0};
  std::uint64_t command_rejections_{0};
  std::uint8_t operation_mode_{jetpilot_msgs::msg::OperationModeState::STOP};
  HostArmState host_arm_state_{HostArmState::disarmed};
  std::optional<jetpilot_msgs::msg::ControlCommand> latest_command_;
  std::optional<SteadyTime> latest_command_time_;
  std::optional<StatusFrame> latest_status_;
  std::optional<SteadyTime> latest_status_time_;
  std::optional<std::uint8_t> last_requested_mode_;
  std::optional<SteadyTime> last_mode_request_time_;
  std::optional<SteadyTime> last_diagnostics_time_;

  rclcpp::Subscription<jetpilot_msgs::msg::ControlCommand>::SharedPtr command_subscription_;
  rclcpp::Subscription<jetpilot_msgs::msg::OperationModeState>::SharedPtr mode_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr steer_offset_inc_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr steer_offset_dec_subscription_;
  rclcpp::Publisher<jetpilot_msgs::msg::OperationModeRequest>::SharedPtr mode_request_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr rc_channels_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr output_channels_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vbec_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_path_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace jetpilot_bridge_interface

#endif  // JETPILOT_BRIDGE_INTERFACE__JETPILOT_BRIDGE_INTERFACE_NODE_HPP_
