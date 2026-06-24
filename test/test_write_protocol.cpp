#include <array>
#include <cstdint>

#include "gtest/gtest.h"

#include "yahboom_10_axis_imu/write_protocol.hpp"

namespace yahboom_10_axis_imu
{
namespace
{

TEST(WriteProtocol, BuildsRegisterWriteFrames)
{
  EXPECT_EQ(
    makeWriteFrame(kRegisterKey, kUnlockValue),
    (WriteFrame{0xFF, 0xAA, 0x69, 0x88, 0xB5}));
  EXPECT_EQ(
    makeWriteFrame(kRegisterRRate, 0x0009),
    (WriteFrame{0xFF, 0xAA, 0x03, 0x09, 0x00}));
  EXPECT_EQ(
    makeWriteFrame(kRegisterBandwidth, kHighestBandwidthValue),
    (WriteFrame{0xFF, 0xAA, 0x1F, 0x00, 0x00}));
  EXPECT_EQ(
    makeWriteFrame(kRegisterSave, kSaveValue),
    (WriteFrame{0xFF, 0xAA, 0x00, 0x00, 0x00}));
}

TEST(WriteProtocol, MapsDocumentedOutputRates)
{
  bool used_fallback = true;
  EXPECT_EQ(outputRateHzToRRateValue(0.2, &used_fallback), 0x0001);
  EXPECT_FALSE(used_fallback);
  EXPECT_EQ(outputRateHzToRRateValue(0.5, &used_fallback), 0x0002);
  EXPECT_FALSE(used_fallback);
  EXPECT_EQ(outputRateHzToRRateValue(1.0, &used_fallback), 0x0003);
  EXPECT_FALSE(used_fallback);
  EXPECT_EQ(outputRateHzToRRateValue(2.0, &used_fallback), 0x0004);
  EXPECT_FALSE(used_fallback);
  EXPECT_EQ(outputRateHzToRRateValue(5.0, &used_fallback), 0x0005);
  EXPECT_FALSE(used_fallback);
  EXPECT_EQ(outputRateHzToRRateValue(10.0, &used_fallback), 0x0006);
  EXPECT_FALSE(used_fallback);
  EXPECT_EQ(outputRateHzToRRateValue(20.0, &used_fallback), 0x0007);
  EXPECT_FALSE(used_fallback);
  EXPECT_EQ(outputRateHzToRRateValue(50.0, &used_fallback), 0x0008);
  EXPECT_FALSE(used_fallback);
  EXPECT_EQ(outputRateHzToRRateValue(100.0, &used_fallback), 0x0009);
  EXPECT_FALSE(used_fallback);
  EXPECT_EQ(outputRateHzToRRateValue(200.0, &used_fallback), 0x000B);
  EXPECT_FALSE(used_fallback);
}

TEST(WriteProtocol, FallsBackToDefaultOutputRate)
{
  bool used_fallback = false;
  EXPECT_EQ(outputRateHzToRRateValue(123.0, &used_fallback), kDefaultRRateValue);
  EXPECT_TRUE(used_fallback);
}

}  // namespace
}  // namespace yahboom_10_axis_imu
