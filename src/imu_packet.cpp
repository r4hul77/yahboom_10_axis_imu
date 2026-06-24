#include "yahboom_10_axis_imu/imu_packet.hpp"

namespace yahboom_10_axis_imu
{

void ImuPacketAssembler::update(const AccelerationFrame & frame)
{
  packet_.linear_acceleration.x = frame.x_mps2;
  packet_.linear_acceleration.y = frame.y_mps2;
  packet_.linear_acceleration.z = frame.z_mps2;
  ready_mask_ = static_cast<uint8_t>(ready_mask_ | kAccelReady);
}

void ImuPacketAssembler::update(const AngularVelocityFrame & frame)
{
  packet_.angular_velocity.x = frame.x_radps;
  packet_.angular_velocity.y = frame.y_radps;
  packet_.angular_velocity.z = frame.z_radps;
  ready_mask_ = static_cast<uint8_t>(ready_mask_ | kGyroReady);
}

void ImuPacketAssembler::update(const QuaternionFrame & frame)
{
  packet_.orientation.w = frame.w;
  packet_.orientation.x = frame.x;
  packet_.orientation.y = frame.y;
  packet_.orientation.z = frame.z;
  ready_mask_ = static_cast<uint8_t>(ready_mask_ | kOrientationReady);
}

bool ImuPacketAssembler::ready() const
{
  return (ready_mask_ & kImuReady) == kImuReady;
}

uint8_t ImuPacketAssembler::readyMask() const
{
  return ready_mask_;
}

const ImuPacket & ImuPacketAssembler::packet() const
{
  return packet_;
}

ImuPacket ImuPacketAssembler::consume()
{
  const ImuPacket completed = packet_;
  ready_mask_ = 0;
  return completed;
}

void ImuPacketAssembler::reset()
{
  packet_ = ImuPacket{};
  ready_mask_ = 0;
}

}  // namespace yahboom_10_axis_imu
