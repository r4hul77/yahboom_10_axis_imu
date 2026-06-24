#include <array>
#include <cmath>
#include <cstdint>
#include <memory>

#include "gtest/gtest.h"

#include "yahboom_10_axis_imu/protocol.hpp"

namespace yahboom_10_axis_imu
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

std::array<uint8_t, 11> makeFrame(uint8_t type, std::array<uint8_t, 8> payload)
{
  std::array<uint8_t, 11> frame{};
  frame[0] = 0x55;
  frame[1] = type;
  for (std::size_t i = 0; i < payload.size(); ++i) {
    frame[i + 2] = payload[i];
  }
  frame[10] = checksum(frame);
  return frame;
}

void feedFrame(ProtocolParser & parser, const std::array<uint8_t, 11> & frame,
  std::unique_ptr<ImuFrameBase> & parsed)
{
  for (const uint8_t byte : frame) {
    auto next = parser.feed(byte);
    if (next) {
      parsed = std::move(next);
    }
  }
}

TEST(ProtocolParser, ParsesAccelerationFrame)
{
  ProtocolParser parser;
  const auto frame = makeFrame(0x51, {0x00, 0x08, 0x00, 0xF8, 0x00, 0x00, 0xE6, 0x09});

  std::unique_ptr<ImuFrameBase> parsed;
  feedFrame(parser, frame, parsed);

  ASSERT_NE(parsed, nullptr);
  const auto * acceleration = dynamic_cast<const AccelerationFrame *>(parsed.get());
  ASSERT_NE(acceleration, nullptr);
  EXPECT_NEAR(acceleration->x_mps2, 9.8, 1e-9);
  EXPECT_NEAR(acceleration->y_mps2, -9.8, 1e-9);
  EXPECT_NEAR(acceleration->z_mps2, 0.0, 1e-9);
  EXPECT_NEAR(acceleration->temperature_c, 25.34, 1e-9);
}

TEST(ProtocolParser, ParsesAngularVelocityFrame)
{
  ProtocolParser parser;
  const auto frame = makeFrame(0x52, {0x00, 0x40, 0x00, 0xC0, 0x00, 0x00, 0xF4, 0x01});

  std::unique_ptr<ImuFrameBase> parsed;
  feedFrame(parser, frame, parsed);

  ASSERT_NE(parsed, nullptr);
  const auto * gyro = dynamic_cast<const AngularVelocityFrame *>(parsed.get());
  ASSERT_NE(gyro, nullptr);
  EXPECT_NEAR(gyro->x_radps, 1000.0 * kPi / 180.0, 1e-9);
  EXPECT_NEAR(gyro->y_radps, -1000.0 * kPi / 180.0, 1e-9);
  EXPECT_NEAR(gyro->z_radps, 0.0, 1e-9);
  EXPECT_NEAR(gyro->voltage_v, 5.0, 1e-9);
}

TEST(ProtocolParser, ParsesQuaternionFrame)
{
  ProtocolParser parser;
  const auto frame = makeFrame(0x59, {0x00, 0x40, 0x00, 0x20, 0x00, 0xE0, 0x00, 0x00});

  std::unique_ptr<ImuFrameBase> parsed;
  feedFrame(parser, frame, parsed);

  ASSERT_NE(parsed, nullptr);
  const auto * quaternion = dynamic_cast<const QuaternionFrame *>(parsed.get());
  ASSERT_NE(quaternion, nullptr);
  EXPECT_NEAR(quaternion->w, 0.5, 1e-9);
  EXPECT_NEAR(quaternion->x, 0.25, 1e-9);
  EXPECT_NEAR(quaternion->y, -0.25, 1e-9);
  EXPECT_NEAR(quaternion->z, 0.0, 1e-9);
}

TEST(ProtocolParser, ParsesMagneticFieldFrame)
{
  ProtocolParser parser;
  const auto frame = makeFrame(0x54, {0x34, 0x12, 0xCC, 0xED, 0x00, 0x80, 0x10, 0x27});

  std::unique_ptr<ImuFrameBase> parsed;
  feedFrame(parser, frame, parsed);

  ASSERT_NE(parsed, nullptr);
  const auto * magnetic = dynamic_cast<const MagneticFieldFrame *>(parsed.get());
  ASSERT_NE(magnetic, nullptr);
  EXPECT_EQ(magnetic->x_raw, 0x1234);
  EXPECT_EQ(magnetic->y_raw, -0x1234);
  EXPECT_EQ(magnetic->z_raw, -32768);
  EXPECT_NEAR(magnetic->temperature_c, 100.0, 1e-9);
}

TEST(ProtocolParser, ParsesPressureAltitudeFrame)
{
  ProtocolParser parser;
  const auto frame = makeFrame(0x56, {0x28, 0x8B, 0x01, 0x00, 0xD0, 0x07, 0x00, 0x00});

  std::unique_ptr<ImuFrameBase> parsed;
  feedFrame(parser, frame, parsed);

  ASSERT_NE(parsed, nullptr);
  const auto * pressure = dynamic_cast<const PressureAltitudeFrame *>(parsed.get());
  ASSERT_NE(pressure, nullptr);
  EXPECT_EQ(pressure->pressure_pa, 101160U);
  EXPECT_EQ(pressure->altitude_cm, 2000);
}

TEST(ProtocolParser, RejectsBadChecksum)
{
  ProtocolParser parser;
  auto frame = makeFrame(0x51, {0x00, 0x08, 0x00, 0xF8, 0x00, 0x00, 0xE6, 0x09});
  frame[10] = static_cast<uint8_t>(frame[10] + 1);

  std::unique_ptr<ImuFrameBase> parsed;
  feedFrame(parser, frame, parsed);

  EXPECT_EQ(parsed, nullptr);
}

TEST(ProtocolParser, ResynchronizesAfterJunkBytes)
{
  ProtocolParser parser;
  EXPECT_EQ(parser.feed(0x01), nullptr);
  EXPECT_EQ(parser.feed(0x02), nullptr);
  EXPECT_EQ(parser.feed(0x55), nullptr);
  EXPECT_EQ(parser.feed(0x99), nullptr);
  EXPECT_EQ(parser.feed(0x00), nullptr);

  const auto frame = makeFrame(0x59, {0x00, 0x40, 0x00, 0x20, 0x00, 0xE0, 0x00, 0x00});

  std::unique_ptr<ImuFrameBase> parsed;
  feedFrame(parser, frame, parsed);

  ASSERT_NE(parsed, nullptr);
  EXPECT_EQ(parsed->type, 0x59);
}

}  // namespace
}  // namespace yahboom_10_axis_imu
