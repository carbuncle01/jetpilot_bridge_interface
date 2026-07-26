#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace jetpilot_bridge_interface
{

constexpr char kProtocolVersion[] = "JPB1";

enum CommandFlag : std::uint16_t
{
  COMMAND_VALID = 1U << 0,
  AUTO_REQUEST = 1U << 1,
};

enum class RcSelector : std::uint8_t
{
  propo = 0,
  automatic = 1,
};

enum class ActivePath : std::uint8_t
{
  disabled = 0,
  manual = 1,
  automatic = 2,
  failsafe = 3,
};

struct CommandFrame
{
  std::uint32_t sequence{0};
  int steering_milli{0};
  int throttle_milli{0};
  int reverse_milli{0};
  int brake_milli{0};
  std::uint16_t flags{0};
};

struct StatusFrame
{
  std::uint32_t sequence{0};
  int rx1_us{0};
  int rx2_us{0};
  int rx3_us{0};
  int servo_output_us{0};
  int esc_output_us{0};
  int vbec_mv{0};
  RcSelector selector{RcSelector::propo};
  ActivePath active_path{ActivePath::disabled};
  std::uint32_t fault_bits{0};
};

std::uint16_t crc16_ccitt(const std::string & data);
std::string encode_command(const CommandFrame & frame);
std::optional<StatusFrame> parse_status(const std::string & line, std::string * error = nullptr);

}  // namespace jetpilot_bridge_interface
