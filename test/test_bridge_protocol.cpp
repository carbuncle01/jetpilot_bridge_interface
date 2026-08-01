#include <iomanip>
#include <sstream>
#include <string>

#include "gtest/gtest.h"
#include "jetpilot_bridge_interface/bridge_protocol.hpp"

namespace jetpilot_bridge_interface
{
namespace
{

std::string with_crc(const std::string & payload)
{
  std::ostringstream output;
  output << payload << ','
         << std::uppercase << std::hex << std::setw(4) << std::setfill('0')
         << crc16_ccitt(payload) << '\n';
  return output.str();
}

TEST(BridgeProtocol, UsesStandardCrc16CcittCheckValue)
{
  EXPECT_EQ(crc16_ccitt("123456789"), 0x29B1U);
}

TEST(BridgeProtocol, EncodesNormalizedCommandFields)
{
  CommandFrame frame;
  frame.sequence = 42;
  frame.steering_milli = -500;
  frame.throttle_milli = 750;
  frame.reverse_milli = 0;
  frame.brake_milli = 100;
  frame.flags = COMMAND_VALID | HOST_REQUEST;

  const auto encoded = encode_command(frame);
  EXPECT_EQ(encoded, with_crc("JPB1,C,42,-500,750,0,100,3"));
}

TEST(BridgeProtocol, RejectsOutOfRangeCommand)
{
  CommandFrame frame;
  frame.steering_milli = 1001;
  EXPECT_THROW(encode_command(frame), std::invalid_argument);
}

TEST(BridgeProtocol, ParsesStatusFrame)
{
  const auto status = parse_status(
    with_crc("JPB1,S,42,1000,1500,2000,1250,1750,6030,1,2,0"));

  ASSERT_TRUE(status);
  EXPECT_EQ(status->sequence, 42U);
  EXPECT_EQ(status->rx1_us, 1000);
  EXPECT_EQ(status->rx2_us, 1500);
  EXPECT_EQ(status->rx3_us, 2000);
  EXPECT_EQ(status->servo_output_us, 1250);
  EXPECT_EQ(status->esc_output_us, 1750);
  EXPECT_EQ(status->vbec_mv, 6030);
  EXPECT_EQ(status->selector, RcSelector::automatic);
  EXPECT_EQ(status->active_path, ActivePath::automatic);
  EXPECT_EQ(status->fault_bits, 0U);
}

TEST(BridgeProtocol, RejectsCorruptedStatusFrame)
{
  auto encoded = with_crc("JPB1,S,42,1000,1500,2000,1250,1750,6030,1,2,0");
  encoded[encoded.find("6030")] = '5';

  std::string error;
  EXPECT_FALSE(parse_status(encoded, &error));
  EXPECT_EQ(error, "CRC mismatch");
}

TEST(BridgeProtocol, RejectsOutOfRangePulse)
{
  std::string error;
  EXPECT_FALSE(
    parse_status(
      with_crc("JPB1,S,42,1000,1500,2500,1250,1750,6030,1,2,0"), &error));
  EXPECT_EQ(error, "status frame value is out of range");
}

}  // namespace
}  // namespace jetpilot_bridge_interface
