#ifndef YAHBOOM_10_AXIS_IMU_WRITE_PROTOCOL_HPP_
#define YAHBOOM_10_AXIS_IMU_WRITE_PROTOCOL_HPP_

#include <array>
#include <cstdint>

namespace yahboom_10_axis_imu
{

constexpr uint8_t kWriteHeader0 = 0xFF;
constexpr uint8_t kWriteHeader1 = 0xAA;
constexpr uint8_t kRegisterSave = 0x00;
constexpr uint8_t kRegisterRRate = 0x03;
constexpr uint8_t kRegisterBandwidth = 0x1F;
constexpr uint8_t kRegisterKey = 0x69;

constexpr uint16_t kUnlockValue = 0xB588;
constexpr uint16_t kSaveValue = 0x0000;
constexpr uint16_t kHighestBandwidthValue = 0x0000;
constexpr double kDefaultOutputRateHz = 100.0;
constexpr uint16_t kDefaultRRateValue = 0x0009;

using WriteFrame = std::array<uint8_t, 5>;

WriteFrame makeWriteFrame(uint8_t register_address, uint16_t value);
uint16_t outputRateHzToRRateValue(double output_rate_hz, bool * used_fallback = nullptr);

}  // namespace yahboom_10_axis_imu

#endif  // YAHBOOM_10_AXIS_IMU_WRITE_PROTOCOL_HPP_
