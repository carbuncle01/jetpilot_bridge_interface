#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "jetpilot_bridge_interface/jetpilot_bridge_interface_node.hpp"

namespace jetpilot_bridge_interface
{

class SerialPort
{
public:
  SerialPort(std::string device, const int baud_rate) : device_(std::move(device))
  {
    const auto baud = termios_baud(baud_rate);
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
      throw std::runtime_error("failed to open " + device_ + ": " + std::strerror(errno));
    }

    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0)
    {
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
    if (::tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
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
    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
      return false;
    }
    if (written < 0)
    {
      throw std::runtime_error("serial write failed: " + std::string(std::strerror(errno)));
    }
    if (static_cast<std::size_t>(written) != line.size())
    {
      throw std::runtime_error("serial write was incomplete");
    }
    return true;
  }

  std::vector<std::string> read_lines()
  {
    char buffer[512];
    while (true)
    {
      const auto count = ::read(fd_, buffer, sizeof(buffer));
      if (count > 0)
      {
        receive_buffer_.append(buffer, static_cast<std::size_t>(count));
        continue;
      }
      if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK)
      {
        break;
      }
      throw std::runtime_error("serial read failed: " + std::string(std::strerror(errno)));
    }

    if (receive_buffer_.size() > 4096)
    {
      receive_buffer_.clear();
      throw std::runtime_error("serial receive buffer exceeded 4096 bytes");
    }

    std::vector<std::string> lines;
    std::size_t newline = 0;
    while ((newline = receive_buffer_.find('\n')) != std::string::npos)
    {
      lines.emplace_back(receive_buffer_.substr(0, newline + 1));
      receive_buffer_.erase(0, newline + 1);
    }
    return lines;
  }

private:
  static speed_t termios_baud(const int baud_rate)
  {
    switch (baud_rate)
    {
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
    if (fd_ >= 0)
    {
      ::close(fd_);
      fd_ = -1;
    }
  }

  std::string device_;
  int fd_{-1};
  std::string receive_buffer_;
};

namespace
{

int normalized_to_milli(const double value, const double minimum, const double maximum)
{
  if (!std::isfinite(value) || value < minimum || value > maximum)
  {
    throw std::invalid_argument("normalized command value is out of range");
  }
  return static_cast<int>(std::lround(value * 1000.0));
}

bool command_values_are_valid(const jetpilot_msgs::msg::ControlCommand & command)
{
  const bool fields_are_finite =
    std::isfinite(command.steering) && std::isfinite(command.throttle) &&
    std::isfinite(command.reverse) && std::isfinite(command.brake);
  return fields_are_finite && command.steering >= -1.0 && command.steering <= 1.0 &&
         command.throttle >= 0.0 && command.throttle <= 1.0 &&
         command.reverse >= 0.0 && command.reverse <= 1.0 &&
         command.brake >= 0.0 && command.brake <= 1.0 &&
         !(command.throttle > 0.0 && command.reverse > 0.0);
}

const char * selector_name(const RcSelector selector)
{
  return selector == RcSelector::automatic ? "HOST" : "PROPO";
}

const char * active_path_name(const ActivePath path)
{
  switch (path)
  {
    case ActivePath::disabled:
      return "DISABLED";
    case ActivePath::manual:
      return "RC";
    case ActivePath::automatic:
      return "HOST";
    case ActivePath::failsafe:
      return "FAILSAFE";
  }
  return "UNKNOWN";
}

const char * arm_state_name(const JetpilotBridgeInterfaceNode::HostArmState state)
{
  switch (state)
  {
    case JetpilotBridgeInterfaceNode::HostArmState::disarmed:
      return "DISARMED";
    case JetpilotBridgeInterfaceNode::HostArmState::arming_neutral:
      return "ARMING_NEUTRAL";
    case JetpilotBridgeInterfaceNode::HostArmState::armed:
      return "ARMED";
  }
  return "UNKNOWN";
}

diagnostic_msgs::msg::KeyValue diagnostic_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  output.value = value;
  return output;
}

}  // namespace

JetpilotBridgeInterfaceNode::JetpilotBridgeInterfaceNode() : Node("jetpilot_bridge_interface_node")
{
  device_ = declare_parameter<std::string>("device", "/dev/ttyACM0");
  baud_rate_ = declare_parameter<int>("baud_rate", 115200);
  command_rate_hz_ = std::max(1.0, declare_parameter<double>("command_rate_hz", 100.0));
  command_timeout_s_ = std::max(0.0, declare_parameter<double>("command_timeout_s", 0.2));
  status_timeout_s_ = std::max(0.0, declare_parameter<double>("status_timeout_s", 0.5));
  reconnect_interval_s_ = std::max(0.1, declare_parameter<double>("reconnect_interval_s", 1.0));
  steering_scale_ = declare_parameter<double>("steering_scale", -1.0);
  if (!std::isfinite(steering_scale_) || std::abs(steering_scale_) > 1.0 ||
    std::abs(steering_scale_) < 1.0e-9)
  {
    throw std::invalid_argument("steering_scale must be finite and in [-1, 1], excluding zero");
  }
  declare_parameter<double>("steering_offset", 0.0);
  declare_parameter<double>("offset_step", 0.01);
  require_status_for_auto_ = declare_parameter<bool>("require_status_for_auto", true);
  publish_mode_request_ = declare_parameter<bool>("publish_mode_request", true);
  frame_id_ = declare_parameter<std::string>("frame_id", "jetpilot_bridge");
  hardware_id_ = declare_parameter<std::string>("hardware_id", "JPBB-01");

  const auto qos_command = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  command_subscription_ = create_subscription<jetpilot_msgs::msg::ControlCommand>(
    "/control_cmd", qos_command,
    [this](const jetpilot_msgs::msg::ControlCommand::SharedPtr message)
    {
      latest_command_ = *message;
      latest_command_time_ = SteadyClock::now();
    });
  steer_offset_inc_subscription_ = create_subscription<std_msgs::msg::Bool>(
    "/steer_offset_inc", 10, [this](const std_msgs::msg::Bool::SharedPtr message)
    {
      if (message->data)
      {
        shift_steering_offset(1.0);
      }
    });
  steer_offset_dec_subscription_ = create_subscription<std_msgs::msg::Bool>(
    "/steer_offset_dec", 10, [this](const std_msgs::msg::Bool::SharedPtr message)
    {
      if (message->data)
      {
        shift_steering_offset(-1.0);
      }
    });
  mode_subscription_ = create_subscription<jetpilot_msgs::msg::OperationModeState>(
    "/operation_mode/state", rclcpp::QoS(1).transient_local().reliable(),
    [this](const jetpilot_msgs::msg::OperationModeState::SharedPtr message)
    {
      const auto previous_mode = operation_mode_;
      operation_mode_ = message->mode;
      if (!is_host_mode(operation_mode_))
      {
        host_arm_state_ = HostArmState::disarmed;
        return;
      }

      // STOP/PROPO -> MANUAL/AUTO is the explicit re-arm gesture.
      const bool board_permits_host =
        !require_status_for_auto_ ||
        (status_is_fresh() && latest_status_ &&
         latest_status_->selector == RcSelector::automatic && latest_status_->fault_bits == 0U);
      if (!is_host_mode(previous_mode) && serial_ && board_permits_host)
      {
        host_arm_state_ = HostArmState::arming_neutral;
      }
    });

  mode_request_publisher_ =
    create_publisher<jetpilot_msgs::msg::OperationModeRequest>("/operation_mode/request", 10);
  rc_channels_publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>("~/rc_channels", 10);
  output_channels_publisher_ =
    create_publisher<std_msgs::msg::Int32MultiArray>("~/output_channels", 10);
  vbec_publisher_ = create_publisher<std_msgs::msg::Float32>("~/vbec_voltage", 10);
  active_path_publisher_ = create_publisher<std_msgs::msg::UInt8>("~/active_path", 10);
  diagnostics_publisher_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

  const auto period = std::chrono::duration<double>(1.0 / command_rate_hz_);
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                             [this]() { update(); });
}

JetpilotBridgeInterfaceNode::~JetpilotBridgeInterfaceNode()
{
  if (serial_)
  {
    try
    {
      CommandFrame stop;
      stop.sequence = sequence_++;
      serial_->write_line(encode_command(stop));
    }
    catch (const std::exception &)
    {
    }
  }
}

void JetpilotBridgeInterfaceNode::update()
{
  ensure_serial_open();
  if (serial_)
  {
    try
    {
      read_status();
      const auto connected_for = serial_open_time_
        ? std::chrono::duration<double>(SteadyClock::now() - *serial_open_time_).count()
        : 0.0;
      if (connected_for > status_timeout_s_ && !status_is_fresh())
      {
        throw std::runtime_error("bridge status timeout");
      }
      write_command();
    }
    catch (const std::exception & error)
    {
      handle_serial_disconnect(error.what());
    }
  }
  publish_diagnostics_if_due();
}

void JetpilotBridgeInterfaceNode::ensure_serial_open()
{
  if (serial_ || std::chrono::steady_clock::now() < next_reconnect_attempt_)
  {
    return;
  }
  try
  {
    serial_ = std::make_unique<SerialPort>(device_, baud_rate_);
    serial_open_time_ = SteadyClock::now();
    latest_status_.reset();
    latest_status_time_.reset();
    host_arm_state_ = HostArmState::disarmed;
    RCLCPP_INFO(get_logger(), "Connected to jetpilot_bridge_board on %s at %d baud",
                device_.c_str(), baud_rate_);
  }
  catch (const std::exception & error)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Bridge connection failed: %s",
                         error.what());
    next_reconnect_attempt_ = std::chrono::steady_clock::now() +
                              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                std::chrono::duration<double>(reconnect_interval_s_));
  }
}

void JetpilotBridgeInterfaceNode::handle_serial_disconnect(const std::string & reason)
{
  RCLCPP_ERROR(get_logger(), "Bridge serial connection lost: %s", reason.c_str());
  serial_.reset();
  serial_open_time_.reset();
  latest_status_.reset();
  latest_status_time_.reset();
  host_arm_state_ = HostArmState::disarmed;
  publish_mode_request(jetpilot_msgs::msg::OperationModeRequest::STOP,
                       "jetpilot_bridge_usb_lost");
  next_reconnect_attempt_ = SteadyClock::now() +
                            std::chrono::duration_cast<SteadyClock::duration>(
                              std::chrono::duration<double>(reconnect_interval_s_));
}

bool JetpilotBridgeInterfaceNode::command_is_fresh() const
{
  return latest_command_ && latest_command_time_ &&
         std::chrono::duration<double>(SteadyClock::now() - *latest_command_time_).count() <=
           command_timeout_s_;
}

bool JetpilotBridgeInterfaceNode::status_is_fresh() const
{
  return latest_status_ && latest_status_time_ &&
         std::chrono::duration<double>(SteadyClock::now() - *latest_status_time_).count() <=
           status_timeout_s_;
}

bool JetpilotBridgeInterfaceNode::is_host_mode(const std::uint8_t mode)
{
  return mode == jetpilot_msgs::msg::OperationModeState::MANUAL ||
         mode == jetpilot_msgs::msg::OperationModeState::AUTO;
}

void JetpilotBridgeInterfaceNode::write_command()
{
  CommandFrame frame;
  frame.sequence = sequence_++;

  const bool fresh_command = command_is_fresh();
  const bool valid_command =
    fresh_command && latest_command_ && command_values_are_valid(*latest_command_);
  const bool host_mode = is_host_mode(operation_mode_);
  if (host_arm_state_ == HostArmState::arming_neutral && host_mode && valid_command)
  {
    // The STM32 only accepts a new arm through a neutral first frame.
    frame.flags = COMMAND_VALID | HOST_REQUEST;
  }
  else if (host_arm_state_ == HostArmState::armed && host_mode && valid_command)
  {
    const auto steering = std::clamp(
      static_cast<double>(latest_command_->steering) * steering_scale_ +
      get_parameter("steering_offset").as_double(), -1.0, 1.0);
    frame.steering_milli = normalized_to_milli(steering, -1.0, 1.0);
    frame.throttle_milli = normalized_to_milli(latest_command_->throttle, 0.0, 1.0);
    frame.reverse_milli = normalized_to_milli(latest_command_->reverse, 0.0, 1.0);
    frame.brake_milli = normalized_to_milli(latest_command_->brake, 0.0, 1.0);
    frame.flags = COMMAND_VALID | HOST_REQUEST;
  }
  else
  {
    // Keep transport alive with an explicit neutral frame, but do not grant
    // host authority. A stale/invalid upstream command also requests STOP.
    frame.flags = COMMAND_VALID;
    if (host_arm_state_ != HostArmState::disarmed && !valid_command)
    {
      host_arm_state_ = HostArmState::disarmed;
      if (fresh_command)
      {
        ++command_rejections_;
      }
      publish_mode_request(jetpilot_msgs::msg::OperationModeRequest::STOP,
                           fresh_command ? "jetpilot_bridge_command_invalid"
                                         : "jetpilot_bridge_command_timeout");
    }
  }

  if (!serial_->write_line(encode_command(frame)))
  {
    ++write_drops_;
  }
}

void JetpilotBridgeInterfaceNode::read_status()
{
  for (const auto & line : serial_->read_lines())
  {
    std::string error;
    const auto status = parse_status(line, &error);
    if (!status)
    {
      ++protocol_errors_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Invalid bridge status: %s",
                           error.c_str());
      continue;
    }
    latest_status_ = *status;
    latest_status_time_ = SteadyClock::now();
    if (status->selector == RcSelector::automatic &&
        status->active_path == ActivePath::automatic && status->fault_bits == 0U &&
        host_arm_state_ == HostArmState::arming_neutral)
    {
      host_arm_state_ = HostArmState::armed;
      RCLCPP_INFO(get_logger(), "JPBB host path armed after neutral handshake");
    }
    else if (status->active_path == ActivePath::manual)
    {
      host_arm_state_ = HostArmState::disarmed;
    }
    else if (status->active_path == ActivePath::failsafe &&
             host_arm_state_ != HostArmState::disarmed)
    {
      host_arm_state_ = HostArmState::disarmed;
      publish_mode_request(jetpilot_msgs::msg::OperationModeRequest::STOP,
                           "jetpilot_bridge_failsafe");
    }
    publish_status(*status);
    publish_mode_request_if_needed(*status);
  }
}

void JetpilotBridgeInterfaceNode::publish_status(const StatusFrame & status)
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

void JetpilotBridgeInterfaceNode::publish_mode_request_if_needed(const StatusFrame & status)
{
  if (!publish_mode_request_)
  {
    return;
  }

  std::optional<std::uint8_t> requested_mode;
  std::string source;
  if (status.active_path == ActivePath::manual && status.selector == RcSelector::propo)
  {
    requested_mode = jetpilot_msgs::msg::OperationModeRequest::PROPO;
    source = "jetpilot_bridge_ch3_propo";
  }
  else if (status.active_path == ActivePath::failsafe)
  {
    requested_mode = jetpilot_msgs::msg::OperationModeRequest::STOP;
    source = "jetpilot_bridge_board_failsafe";
  }
  if (!requested_mode)
  {
    return;
  }

  const auto current_time = SteadyClock::now();
  const bool mode_changed = !last_requested_mode_ || *last_requested_mode_ != *requested_mode;
  const bool refresh_due =
    !last_mode_request_time_ ||
    std::chrono::duration<double>(current_time - *last_mode_request_time_).count() >= 1.0;
  if (!mode_changed && !refresh_due)
  {
    return;
  }

  publish_mode_request(*requested_mode, source);
  last_requested_mode_ = *requested_mode;
  last_mode_request_time_ = current_time;
}

void JetpilotBridgeInterfaceNode::publish_mode_request(const std::uint8_t mode,
                                                        const std::string & source)
{
  jetpilot_msgs::msg::OperationModeRequest request;
  request.header.stamp = now();
  request.header.frame_id = frame_id_;
  request.mode = mode;
  request.source = source;
  mode_request_publisher_->publish(request);
}

void JetpilotBridgeInterfaceNode::publish_diagnostics_if_due()
{
  const auto current_time = SteadyClock::now();
  if (last_diagnostics_time_ &&
      std::chrono::duration<double>(current_time - *last_diagnostics_time_).count() < 1.0)
  {
    return;
  }
  last_diagnostics_time_ = current_time;

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "jetpilot_bridge_interface";
  status.hardware_id = hardware_id_;
  if (!serial_)
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "USB serial disconnected";
  }
  else if (!status_is_fresh())
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "bridge status timeout";
  }
  else if (latest_status_->fault_bits != 0U)
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "bridge reported a fault";
  }
  else
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "bridge healthy";
  }

  status.values.push_back(diagnostic_value("device", device_));
  status.values.push_back(diagnostic_value("protocol", kProtocolVersion));
  status.values.push_back(diagnostic_value("command_fresh", command_is_fresh() ? "true" : "false"));
  status.values.push_back(diagnostic_value("status_fresh", status_is_fresh() ? "true" : "false"));
  status.values.push_back(diagnostic_value("protocol_errors", std::to_string(protocol_errors_)));
  status.values.push_back(diagnostic_value("write_drops", std::to_string(write_drops_)));
  status.values.push_back(
    diagnostic_value("command_rejections", std::to_string(command_rejections_)));
  status.values.push_back(diagnostic_value("steering_scale", std::to_string(steering_scale_)));
  status.values.push_back(diagnostic_value(
    "steering_offset", std::to_string(get_parameter("steering_offset").as_double())));
  status.values.push_back(diagnostic_value("host_arm_state", arm_state_name(host_arm_state_)));
  status.values.push_back(diagnostic_value(
    "require_status_for_host", require_status_for_auto_ ? "true" : "false"));
  if (latest_status_)
  {
    status.values.push_back(diagnostic_value("selector", selector_name(latest_status_->selector)));
    status.values.push_back(
      diagnostic_value("active_path", active_path_name(latest_status_->active_path)));
    status.values.push_back(diagnostic_value(
      "vbec_v", std::to_string(static_cast<double>(latest_status_->vbec_mv) / 1000.0)));
    std::ostringstream fault_bits;
    fault_bits << "0x" << std::hex << latest_status_->fault_bits;
    status.values.push_back(diagnostic_value("fault_bits", fault_bits.str()));
  }

  array.status.push_back(status);
  diagnostics_publisher_->publish(array);
}

void JetpilotBridgeInterfaceNode::shift_steering_offset(const double direction)
{
  const auto current_offset = get_parameter("steering_offset").as_double();
  const auto offset_step = std::max(0.0, get_parameter("offset_step").as_double());
  const auto next_offset = std::clamp(current_offset + offset_step * direction, -1.0, 1.0);
  set_parameter(rclcpp::Parameter("steering_offset", next_offset));
  RCLCPP_INFO(get_logger(), "Steering offset set to %.3f", next_offset);
}

}  // namespace jetpilot_bridge_interface
