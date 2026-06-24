#ifndef YAHBOOM_10_AXIS_IMU_IMU_PACKET_HPP_
#define YAHBOOM_10_AXIS_IMU_IMU_PACKET_HPP_

#include <cstdint>

#include "yahboom_10_axis_imu/protocol.hpp"

namespace yahboom_10_axis_imu
{

constexpr uint8_t kAccelReady = 0b001;
constexpr uint8_t kGyroReady = 0b010;
constexpr uint8_t kOrientationReady = 0b100;
constexpr uint8_t kImuReady = 0b111;

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ImuPacket
{
  Vector3 linear_acceleration;
  Vector3 angular_velocity;
  Quaternion orientation;
};

class ImuPacketAssembler
{
public:
  void update(const AccelerationFrame & frame);
  void update(const AngularVelocityFrame & frame);
  void update(const QuaternionFrame & frame);

  bool ready() const;
  uint8_t readyMask() const;
  const ImuPacket & packet() const;
  ImuPacket consume();
  void reset();

private:
  ImuPacket packet_;
  uint8_t ready_mask_{0};
};

}  // namespace yahboom_10_axis_imu

#endif  // YAHBOOM_10_AXIS_IMU_IMU_PACKET_HPP_
