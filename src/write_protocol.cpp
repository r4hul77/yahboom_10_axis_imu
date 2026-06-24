#include "yahboom_10_axis_imu/write_protocol.hpp"

#include <cmath>

namespace yahboom_10_axis_imu
{
namespace
{

bool nearlyEqual(double lhs, double rhs)
{
  return std::fabs(lhs - rhs) < 1e-6;
}

}  // namespace

WriteFrame makeWriteFrame(uint8_t register_address, uint16_t value)
{
  return WriteFrame{
    kWriteHeader0,
    kWriteHeader1,
    register_address,
    static_cast<uint8_t>(value & 0xFF),
    static_cast<uint8_t>((value >> 8) & 0xFF)};
}

uint16_t outputRateHzToRRateValue(double output_rate_hz, bool * used_fallback)
{
  if (used_fallback != nullptr) {
    *used_fallback = false;
  }

  if (nearlyEqual(output_rate_hz, 0.2)) {
    return 0x0001;
  }
  if (nearlyEqual(output_rate_hz, 0.5)) {
    return 0x0002;
  }
  if (nearlyEqual(output_rate_hz, 1.0)) {
    return 0x0003;
  }
  if (nearlyEqual(output_rate_hz, 2.0)) {
    return 0x0004;
  }
  if (nearlyEqual(output_rate_hz, 5.0)) {
    return 0x0005;
  }
  if (nearlyEqual(output_rate_hz, 10.0)) {
    return 0x0006;
  }
  if (nearlyEqual(output_rate_hz, 20.0)) {
    return 0x0007;
  }
  if (nearlyEqual(output_rate_hz, 50.0)) {
    return 0x0008;
  }
  if (nearlyEqual(output_rate_hz, 100.0)) {
    return 0x0009;
  }
  if (nearlyEqual(output_rate_hz, 200.0)) {
    return 0x000B;
  }

  if (used_fallback != nullptr) {
    *used_fallback = true;
  }
  return kDefaultRRateValue;
}

}  // namespace yahboom_10_axis_imu
