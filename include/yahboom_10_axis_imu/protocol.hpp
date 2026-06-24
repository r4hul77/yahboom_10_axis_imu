#ifndef YAHBOOM_10_AXIS_IMU_PROTOCOL_HPP_
#define YAHBOOM_10_AXIS_IMU_PROTOCOL_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>

namespace yahboom_10_axis_imu
{

enum class FrameType : uint8_t
{
  kTime = 0x50,
  kAcceleration = 0x51,
  kAngularVelocity = 0x52,
  kAngle = 0x53,
  kMagneticField = 0x54,
  kPortStatus = 0x55,
  kPressureAltitude = 0x56,
  kLatitudeLongitude = 0x57,
  kGpsData = 0x58,
  kQuaternion = 0x59,
  kGpsAccuracy = 0x5A,
  kReadRegister = 0x5F,
};

struct ImuFrameBase
{
  explicit ImuFrameBase(uint8_t frame_type);
  virtual ~ImuFrameBase() = default;

  uint8_t type;
  std::chrono::steady_clock::time_point receive_time;
  std::array<uint8_t, 8> raw_payload{};
  bool checksum_valid{false};
};

struct TimeFrame : public ImuFrameBase
{
  TimeFrame();

  uint8_t year{0};
  uint8_t month{0};
  uint8_t day{0};
  uint8_t hour{0};
  uint8_t minute{0};
  uint8_t second{0};
  uint16_t millisecond{0};
};

struct AccelerationFrame : public ImuFrameBase
{
  AccelerationFrame();

  double x_mps2{0.0};
  double y_mps2{0.0};
  double z_mps2{0.0};
  double temperature_c{0.0};
};

struct AngularVelocityFrame : public ImuFrameBase
{
  AngularVelocityFrame();

  double x_radps{0.0};
  double y_radps{0.0};
  double z_radps{0.0};
  double voltage_v{0.0};
};

struct AngleFrame : public ImuFrameBase
{
  AngleFrame();

  double roll_rad{0.0};
  double pitch_rad{0.0};
  double yaw_rad{0.0};
  uint16_t version{0};
};

struct MagneticFieldFrame : public ImuFrameBase
{
  MagneticFieldFrame();

  int16_t x_raw{0};
  int16_t y_raw{0};
  int16_t z_raw{0};
  double temperature_c{0.0};
};

struct PortStatusFrame : public ImuFrameBase
{
  PortStatusFrame();

  std::array<uint16_t, 4> status{};
};

struct PressureAltitudeFrame : public ImuFrameBase
{
  PressureAltitudeFrame();

  uint32_t pressure_pa{0};
  int32_t altitude_cm{0};
};

struct LatitudeLongitudeFrame : public ImuFrameBase
{
  LatitudeLongitudeFrame();

  int32_t longitude_raw{0};
  int32_t latitude_raw{0};
};

struct GpsDataFrame : public ImuFrameBase
{
  GpsDataFrame();

  double gps_height_m{0.0};
  double gps_yaw_deg{0.0};
  double ground_speed_kph{0.0};
};

struct QuaternionFrame : public ImuFrameBase
{
  QuaternionFrame();

  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct GpsAccuracyFrame : public ImuFrameBase
{
  GpsAccuracyFrame();

  uint16_t satellite_count{0};
  double pdop{0.0};
  double hdop{0.0};
  double vdop{0.0};
};

struct ReadRegisterFrame : public ImuFrameBase
{
  ReadRegisterFrame();

  std::array<uint16_t, 4> registers{};
};

class ProtocolParser
{
public:
  std::unique_ptr<ImuFrameBase> feed(uint8_t byte);
  void reset();

private:
  static constexpr uint8_t kHeader = 0x55;
  static constexpr std::size_t kFrameSize = 11;

  std::unique_ptr<ImuFrameBase> tryParseBufferedFrame();
  std::deque<uint8_t> buffer_;
};

bool isKnownFrameType(uint8_t type);
uint8_t checksum(const std::array<uint8_t, 11> & frame);
std::unique_ptr<ImuFrameBase> parseFrame(const std::array<uint8_t, 11> & frame);

}  // namespace yahboom_10_axis_imu

#endif  // YAHBOOM_10_AXIS_IMU_PROTOCOL_HPP_
