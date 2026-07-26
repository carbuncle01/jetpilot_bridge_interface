#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "jetpilot_bridge_interface/bridge_protocol.hpp"
#include "jetpilot_msgs/msg/control_command.hpp"
#include "jetpilot_msgs/msg/operation_mode_request.hpp"
#include "jetpilot_msgs/msg/operation_mode_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace jetpilot_bridge_interface
{
namespace
{

class SerialPort
{
public:
  SerialPort(std::string device, const int baud_rate)
  : device_(std::move(device))
  {
    const auto baud = termios_baud(baud_rate);
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("failed to open " + device_ + ": " + std::strerror(errno));
    }

    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
      const auto message = "failed to read serial settings: " + std::string(std::strerror(errno));
      close();
      throw std::runtime_error(message);
    }

    ::cfmakeraw(&tty);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    ::cfsetispeed(&tty, baud);
    ::cfsetospeed(&tty, baud);
    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
      const auto message = "failed to apply serial settings: " + std::string(std::strerror(errno));
      close();
      throw std::runtime_error(message);
    }
    ::tcflush(fd_, TCIOFLUSH);
  }

  ~SerialPort()
  {
    close();
  }

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  bool write_line(const std::string & line)
  {
    const auto written = ::write(fd_, line.data(), line.size());
    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return false;
    }
    if (written < 0) {
      throw std::runtime_error("serial write failed: " + std::string(std::strerror(errno)));
    }
    if (static_cast<std::size_t>(written) != line.size()) {
      throw std::runtime_error("serial write was incomplete");
    }
    return true;
  }

  std::vector<std::string> read_lines()
  {
    char buffer[512];
    while (true) {
      const auto count = ::read(fd_, buffer, sizeof(buffer));
      if (count > 0) {
        receive_buffer_.append(buffer, static_cast<std::size_t>(count));
        continue;
      }
      if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      throw std::runtime_error("serial read failed: " + std::string(std::strerror(errno)));
    }

    if (receive_buffer_.size() > 4096) {
      receive_buffer_.clear();
      throw std::runtime_error("serial receive buffer exceeded 4096 bytes");
    }

    std::vector<std::string> lines;
    std::size_t newline = 0;
    while ((newline = receive_buffer_.find('\n')) != std::string::npos) {
      lines.emplace_back(receive_buffer_.substr(0, newline + 1));
      receive_buffer_.erase(0, newline + 1);
    }
    return lines;
  }

private:
  static speed_t termios_baud(const int baud_rate)
  {
    switch (baud_rate) {
      case 115200:
        return B115200;
#ifdef B230400
      case 230400:
        return B230400;
#endif
#ifdef B460800
      case 460800:
        return B460800;
#endif
#ifdef B921600
      case 921600:
        return B921600;
#endif
      default:
        throw std::invalid_argument("unsupported serial baud rate: " + std::to_string(baud_rate));
    }
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  std::string device_;
  int fd_{-1};
  std::string receive_buffer_;
};

int normalized_to_milli(const double value, const double minimum, const double maximum)
{
  return static_cast<int>(std::lround(std::clamp(value, minimum, maximum) * 1000.0));
}

const char * selector_name(const RcSelector selector)
{
  return selector == RcSelector::automatic ? "AUTO" : "PROPO";
}

const char * active_path_name(const ActivePath path)
{
  switch (path) {
    case ActivePath::disabled:
      return "DISABLED";
    case ActivePath::manual:
      return "MANUAL";
    case ActivePath::automatic:
      return "AUTO";
    case ActivePath::failsafe:
      return "FAILSAFE";
  }
  return "UNKNOWN";
}

diagnostic_msgs::msg::KeyValue diagnostic_value(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

}  // namespace

class JetpilotBridgeInterfaceNode : public rclcpp::Node
{
public:
  JetpilotBridgeInterfaceNode()
  : Node("jetpilot_bridge_interface_node")
  {
    device_ = declare_parameter<std::string>("device", "/dev/ttyACM0");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    command_rate_hz_ = std::max(1.0, declare_parameter<double>("command_rate_hz", 100.0));
    command_timeout_s_ = std::max(0.0, declare_parameter<double>("command_timeout_s", 0.3));
    status_timeout_s_ = std::max(0.0, declare_parameter<double>("status_timeout_s", 0.5));
    reconnect_interval_s_ =
      std::max(0.1, declare_parameter<double>("reconnect_interval_s", 1.0));
    require_status_for_auto_ = declare_parameter<bool>("require_status_for_auto", true);
    publish_mode_request_ = declare_parameter<bool>("publish_mode_request", true);
    frame_id_ = declare_parameter<std::string>("frame_id", "jetpilot_bridge");
    hardware_id_ = declare_parameter<std::string>("hardware_id", "JPBB-01");

    const auto qos_command = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    command_subscription_ = create_subscription<jetpilot_msgs::msg::ControlCommand>(
      "/control_cmd", qos_command,
      [this](const jetpilot_msgs::msg::ControlCommand::SharedPtr message) {
        latest_command_ = *message;
        latest_command_time_ = now();
      });
    mode_subscription_ = create_subscription<jetpilot_msgs::msg::OperationModeState>(
      "/operation_mode/state", rclcpp::QoS(1).transient_local().reliable(),
      [this](const jetpilot_msgs::msg::OperationModeState::SharedPtr message) {
        operation_mode_ = message->mode;
      });

    mode_request_publisher_ = create_publisher<jetpilot_msgs::msg::OperationModeRequest>(
      "/operation_mode/request", 10);
    rc_channels_publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      "~/rc_channels", 10);
    output_channels_publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      "~/output_channels", 10);
    vbec_publisher_ = create_publisher<std_msgs::msg::Float32>("~/vbec_voltage", 10);
    active_path_publisher_ = create_publisher<std_msgs::msg::UInt8>("~/active_path", 10);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);

    const auto period = std::chrono::duration<double>(1.0 / command_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { update(); });
  }

  ~JetpilotBridgeInterfaceNode() override
  {
    if (serial_) {
      try {
        CommandFrame stop;
        stop.sequence = sequence_++;
        serial_->write_line(encode_command(stop));
      } catch (const std::exception &) {
      }
    }
  }

private:
  void update()
  {
    ensure_serial_open();
    if (serial_) {
      try {
        read_status();
        write_command();
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Bridge serial connection lost: %s", error.what());
        serial_.reset();
        next_reconnect_attempt_ =
          std::chrono::steady_clock::now() + std::chrono::duration_cast<
          std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(reconnect_interval_s_));
      }
    }
    publish_diagnostics_if_due();
  }

  void ensure_serial_open()
  {
    if (serial_ || std::chrono::steady_clock::now() < next_reconnect_attempt_) {
      return;
    }
    try {
      serial_ = std::make_unique<SerialPort>(device_, baud_rate_);
      RCLCPP_INFO(
        get_logger(), "Connected to jetpilot_bridge_board on %s at %d baud",
        device_.c_str(), baud_rate_);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Bridge connection failed: %s", error.what());
      next_reconnect_attempt_ =
        std::chrono::steady_clock::now() + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(reconnect_interval_s_));
    }
  }

  bool command_is_fresh() const
  {
    return latest_command_ && latest_command_time_ &&
           (now() - *latest_command_time_).seconds() <= command_timeout_s_;
  }

  bool status_is_fresh() const
  {
    return latest_status_ && latest_status_time_ &&
           (now() - *latest_status_time_).seconds() <= status_timeout_s_;
  }

  void write_command()
  {
    CommandFrame frame;
    frame.sequence = sequence_++;

    if (command_is_fresh()) {
      frame.steering_milli = normalized_to_milli(latest_command_->steering, -1.0, 1.0);
      frame.throttle_milli = normalized_to_milli(latest_command_->throttle, 0.0, 1.0);
      frame.reverse_milli = normalized_to_milli(latest_command_->reverse, 0.0, 1.0);
      frame.brake_milli = normalized_to_milli(latest_command_->brake, 0.0, 1.0);
      frame.flags |= COMMAND_VALID;
    }

    const bool auto_mode =
      operation_mode_ == jetpilot_msgs::msg::OperationModeState::AUTO;
    if (auto_mode && (!require_status_for_auto_ || status_is_fresh())) {
      frame.flags |= AUTO_REQUEST;
    }

    if (!serial_->write_line(encode_command(frame))) {
      ++write_drops_;
    }
  }

  void read_status()
  {
    for (const auto & line : serial_->read_lines()) {
      std::string error;
      const auto status = parse_status(line, &error);
      if (!status) {
        ++protocol_errors_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Invalid bridge status: %s", error.c_str());
        continue;
      }
      latest_status_ = *status;
      latest_status_time_ = now();
      publish_status(*status);
      publish_mode_request_if_needed(*status);
    }
  }

  void publish_status(const StatusFrame & status)
  {
    std_msgs::msg::Int32MultiArray rc_channels;
    rc_channels.data = {status.rx1_us, status.rx2_us, status.rx3_us};
    rc_channels_publisher_->publish(rc_channels);

    std_msgs::msg::Int32MultiArray output_channels;
    output_channels.data = {status.servo_output_us, status.esc_output_us};
    output_channels_publisher_->publish(output_channels);

    std_msgs::msg::Float32 vbec;
    vbec.data = static_cast<float>(status.vbec_mv) / 1000.0F;
    vbec_publisher_->publish(vbec);

    std_msgs::msg::UInt8 active_path;
    active_path.data = static_cast<std::uint8_t>(status.active_path);
    active_path_publisher_->publish(active_path);
  }

  void publish_mode_request_if_needed(const StatusFrame & status)
  {
    if (!publish_mode_request_) {
      return;
    }

    const auto current_time = now();
    const bool selector_changed =
      !last_requested_selector_ || *last_requested_selector_ != status.selector;
    const bool refresh_due =
      !last_mode_request_time_ || (current_time - *last_mode_request_time_).seconds() >= 1.0;
    if (!selector_changed && !refresh_due) {
      return;
    }

    jetpilot_msgs::msg::OperationModeRequest request;
    request.header.stamp = current_time;
    request.header.frame_id = frame_id_;
    request.mode = status.selector == RcSelector::automatic ?
      jetpilot_msgs::msg::OperationModeRequest::AUTO :
      jetpilot_msgs::msg::OperationModeRequest::PROPO;
    request.source = "jetpilot_bridge_ch3";
    mode_request_publisher_->publish(request);
    last_requested_selector_ = status.selector;
    last_mode_request_time_ = current_time;
  }

  void publish_diagnostics_if_due()
  {
    const auto current_time = now();
    if (
      last_diagnostics_time_ &&
      (current_time - *last_diagnostics_time_).seconds() < 1.0)
    {
      return;
    }
    last_diagnostics_time_ = current_time;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = current_time;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "jetpilot_bridge_interface";
    status.hardware_id = hardware_id_;
    if (!serial_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "USB serial disconnected";
    } else if (!status_is_fresh()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "bridge status timeout";
    } else if (latest_status_->fault_bits != 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "bridge reported a fault";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "bridge healthy";
    }

    status.values.push_back(diagnostic_value("device", device_));
    status.values.push_back(diagnostic_value("protocol", kProtocolVersion));
    status.values.push_back(diagnostic_value("command_fresh", command_is_fresh() ? "true" : "false"));
    status.values.push_back(diagnostic_value("status_fresh", status_is_fresh() ? "true" : "false"));
    status.values.push_back(diagnostic_value("protocol_errors", std::to_string(protocol_errors_)));
    status.values.push_back(diagnostic_value("write_drops", std::to_string(write_drops_)));
    if (latest_status_) {
      status.values.push_back(
        diagnostic_value("selector", selector_name(latest_status_->selector)));
      status.values.push_back(
        diagnostic_value("active_path", active_path_name(latest_status_->active_path)));
      status.values.push_back(
        diagnostic_value("vbec_v", std::to_string(
          static_cast<double>(latest_status_->vbec_mv) / 1000.0)));
      std::ostringstream fault_bits;
      fault_bits << "0x" << std::hex << latest_status_->fault_bits;
      status.values.push_back(diagnostic_value("fault_bits", fault_bits.str()));
    }

    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  std::string device_;
  int baud_rate_{115200};
  double command_rate_hz_{100.0};
  double command_timeout_s_{0.3};
  double status_timeout_s_{0.5};
  double reconnect_interval_s_{1.0};
  bool require_status_for_auto_{true};
  bool publish_mode_request_{true};
  std::string frame_id_;
  std::string hardware_id_;

  std::unique_ptr<SerialPort> serial_;
  std::chrono::steady_clock::time_point next_reconnect_attempt_{};
  std::uint32_t sequence_{0};
  std::uint64_t protocol_errors_{0};
  std::uint64_t write_drops_{0};
  std::uint8_t operation_mode_{jetpilot_msgs::msg::OperationModeState::STOP};
  std::optional<jetpilot_msgs::msg::ControlCommand> latest_command_;
  std::optional<rclcpp::Time> latest_command_time_;
  std::optional<StatusFrame> latest_status_;
  std::optional<rclcpp::Time> latest_status_time_;
  std::optional<RcSelector> last_requested_selector_;
  std::optional<rclcpp::Time> last_mode_request_time_;
  std::optional<rclcpp::Time> last_diagnostics_time_;

  rclcpp::Subscription<jetpilot_msgs::msg::ControlCommand>::SharedPtr command_subscription_;
  rclcpp::Subscription<jetpilot_msgs::msg::OperationModeState>::SharedPtr mode_subscription_;
  rclcpp::Publisher<jetpilot_msgs::msg::OperationModeRequest>::SharedPtr
    mode_request_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr rc_channels_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr output_channels_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vbec_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr active_path_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace jetpilot_bridge_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<jetpilot_bridge_interface::JetpilotBridgeInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
