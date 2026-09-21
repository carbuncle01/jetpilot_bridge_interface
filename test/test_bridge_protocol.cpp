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

TEST(BridgeProtocol, ConvertsPropoPwmUsingHostInverseMapping)
{
  const PropoCalibration calibration;

  const auto neutral = propo_pwm_to_command(1500, 1500, calibration);
  ASSERT_TRUE(neutral);
  EXPECT_FLOAT_EQ(neutral->steering, 0.0F);
  EXPECT_FLOAT_EQ(neutral->throttle, 0.0F);
  EXPECT_FLOAT_EQ(neutral->reverse, 0.0F);

  const auto left_forward = propo_pwm_to_command(1000, 1000, calibration);
  ASSERT_TRUE(left_forward);
  EXPECT_FLOAT_EQ(left_forward->steering, 1.0F);
  EXPECT_FLOAT_EQ(left_forward->throttle, 1.0F);
  EXPECT_FLOAT_EQ(left_forward->reverse, 0.0F);

  const auto right_reverse = propo_pwm_to_command(2000, 2000, calibration);
  ASSERT_TRUE(right_reverse);
  EXPECT_FLOAT_EQ(right_reverse->steering, -1.0F);
  EXPECT_FLOAT_EQ(right_reverse->throttle, 0.0F);
  EXPECT_FLOAT_EQ(right_reverse->reverse, 1.0F);
}

TEST(BridgeProtocol, PreservesMeasuredPropoAuthorityRelativeToHostPwm)
{
  const auto command = propo_pwm_to_command(1073, 1078, PropoCalibration{});
  ASSERT_TRUE(command);
  EXPECT_NEAR(command->steering, 0.854F, 1.0e-6F);
  EXPECT_NEAR(command->throttle, 0.844F, 1.0e-6F);
}

TEST(BridgeProtocol, RejectsInvalidPropoCalibrationAndMissingPulse)
{
  auto calibration = PropoCalibration{};
  calibration.throttle_forward_us = calibration.throttle_neutral_us;
  EXPECT_FALSE(propo_calibration_is_valid(calibration));
  EXPECT_FALSE(propo_pwm_to_command(1500, 1500, calibration));
  EXPECT_FALSE(propo_pwm_to_command(0, 1500, PropoCalibration{}));
}

}  // namespace
}  // namespace jetpilot_bridge_interface
