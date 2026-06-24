#include "gtest/gtest.h"

#include "yahboom_10_axis_imu/imu_packet.hpp"

namespace yahboom_10_axis_imu
{
namespace
{

TEST(ImuPacketAssembler, WaitsForCompleteThreeBitMask)
{
  ImuPacketAssembler assembler;

  AccelerationFrame acceleration;
  acceleration.x_mps2 = 1.0;
  acceleration.y_mps2 = 2.0;
  acceleration.z_mps2 = 3.0;
  assembler.update(acceleration);

  EXPECT_EQ(assembler.readyMask(), kAccelReady);
  EXPECT_FALSE(assembler.ready());

  AngularVelocityFrame angular_velocity;
  angular_velocity.x_radps = 4.0;
  angular_velocity.y_radps = 5.0;
  angular_velocity.z_radps = 6.0;
  assembler.update(angular_velocity);

  EXPECT_EQ(assembler.readyMask(), static_cast<uint8_t>(kAccelReady | kGyroReady));
  EXPECT_FALSE(assembler.ready());

  QuaternionFrame quaternion;
  quaternion.w = 0.7;
  quaternion.x = 0.1;
  quaternion.y = 0.2;
  quaternion.z = 0.3;
  assembler.update(quaternion);

  EXPECT_EQ(assembler.readyMask(), kImuReady);
  EXPECT_TRUE(assembler.ready());
}

TEST(ImuPacketAssembler, ConsumesAndClearsMask)
{
  ImuPacketAssembler assembler;

  AccelerationFrame acceleration;
  acceleration.x_mps2 = 1.0;
  assembler.update(acceleration);

  AngularVelocityFrame angular_velocity;
  angular_velocity.y_radps = 2.0;
  assembler.update(angular_velocity);

  QuaternionFrame quaternion;
  quaternion.z = 3.0;
  assembler.update(quaternion);

  ASSERT_TRUE(assembler.ready());
  const ImuPacket packet = assembler.consume();

  EXPECT_EQ(assembler.readyMask(), 0);
  EXPECT_FALSE(assembler.ready());
  EXPECT_DOUBLE_EQ(packet.linear_acceleration.x, 1.0);
  EXPECT_DOUBLE_EQ(packet.angular_velocity.y, 2.0);
  EXPECT_DOUBLE_EQ(packet.orientation.z, 3.0);
}

TEST(ImuPacketAssembler, ResetClearsPacketAndMask)
{
  ImuPacketAssembler assembler;
  AccelerationFrame acceleration;
  acceleration.x_mps2 = 42.0;
  assembler.update(acceleration);

  assembler.reset();

  EXPECT_EQ(assembler.readyMask(), 0);
  EXPECT_FALSE(assembler.ready());
  EXPECT_DOUBLE_EQ(assembler.packet().linear_acceleration.x, 0.0);
}

}  // namespace
}  // namespace yahboom_10_axis_imu
