#include "yahboom_10_axis_imu/protocol.hpp"

#include <algorithm>
#include <cmath>

namespace yahboom_10_axis_imu
{
namespace
{

constexpr double kGravityMps2 = 9.8;
constexpr double kPi = 3.14159265358979323846;

int16_t int16FromLe(uint8_t low, uint8_t high)
{
  return static_cast<int16_t>(static_cast<uint16_t>(low) |
    (static_cast<uint16_t>(high) << 8));
}

uint16_t uint16FromLe(uint8_t low, uint8_t high)
{
  return static_cast<uint16_t>(static_cast<uint16_t>(low) |
    (static_cast<uint16_t>(high) << 8));
}

uint32_t uint32FromLe(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  return static_cast<uint32_t>(b0) |
    (static_cast<uint32_t>(b1) << 8) |
    (static_cast<uint32_t>(b2) << 16) |
    (static_cast<uint32_t>(b3) << 24);
}

int32_t int32FromLe(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
  return static_cast<int32_t>(uint32FromLe(b0, b1, b2, b3));
}

void populateBase(ImuFrameBase & parsed, const std::array<uint8_t, 11> & frame)
{
  std::copy(frame.begin() + 2, frame.begin() + 10, parsed.raw_payload.begin());
  parsed.receive_time = std::chrono::steady_clock::now();
  parsed.checksum_valid = true;
}

double scaledInt16(uint8_t low, uint8_t high, double scale)
{
  return static_cast<double>(int16FromLe(low, high)) / 32768.0 * scale;
}

}  // namespace

ImuFrameBase::ImuFrameBase(uint8_t frame_type)
: type(frame_type),
  receive_time(std::chrono::steady_clock::now())
{
}

TimeFrame::TimeFrame()
: ImuFrameBase(static_cast<uint8_t>(FrameType::kTime))
{
}

AccelerationFrame::AccelerationFrame()
: ImuFrameBase(static_cast<uint8_t>(FrameType::kAcceleration))
{
}

AngularVelocityFrame::AngularVelocityFrame()
: ImuFrameBase(static_cast<uint8_t>(FrameType::kAngularVelocity))
{
}

AngleFrame::AngleFrame()
: ImuFrameBase(static_cast<uint8_t>(FrameType::kAngle))
{
}

MagneticFieldFrame::MagneticFieldFrame()
: ImuFrameBase(static_cast<uint8_t>(FrameType::kMagneticField))
{
}

PortStatusFrame::PortStatusFrame()
: ImuFrameBase(static_cast<uint8_t>(FrameType::kPortStatus))
{
}

PressureAltitudeFrame::PressureAltitudeFrame()
: ImuFrameBase(static_cast<uint8_t>(FrameType::kPressureAltitude))
{
}

LatitudeLongitudeFrame::LatitudeLongitudeFrame()
: ImuFrameBase(static_cast<uint8_t>(FrameType::kLatitudeLongitude))
{
}

GpsDataFrame::GpsDataFrame()
: ImuFrameBase(static_cast<uint8_t>(FrameType::kGpsData))
{
}

QuaternionFrame::QuaternionFrame()
: ImuFrameBase(static_cast<uint8_t>(FrameType::kQuaternion))
{
}

GpsAccuracyFrame::GpsAccuracyFrame()
: ImuFrameBase(static_cast<uint8_t>(FrameType::kGpsAccuracy))
{
}

ReadRegisterFrame::ReadRegisterFrame()
: ImuFrameBase(static_cast<uint8_t>(FrameType::kReadRegister))
{
}

bool isKnownFrameType(uint8_t type)
{
  switch (static_cast<FrameType>(type)) {
    case FrameType::kTime:
    case FrameType::kAcceleration:
    case FrameType::kAngularVelocity:
    case FrameType::kAngle:
    case FrameType::kMagneticField:
    case FrameType::kPortStatus:
    case FrameType::kPressureAltitude:
    case FrameType::kLatitudeLongitude:
    case FrameType::kGpsData:
    case FrameType::kQuaternion:
    case FrameType::kGpsAccuracy:
    case FrameType::kReadRegister:
      return true;
    default:
      return false;
  }
}

uint8_t checksum(const std::array<uint8_t, 11> & frame)
{
  uint16_t sum = 0;
  for (std::size_t i = 0; i < 10; ++i) {
    sum = static_cast<uint16_t>(sum + frame[i]);
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

std::unique_ptr<ImuFrameBase> parseFrame(const std::array<uint8_t, 11> & frame)
{
  if (frame[0] != 0x55 || checksum(frame) != frame[10] || !isKnownFrameType(frame[1])) {
    return nullptr;
  }

  switch (static_cast<FrameType>(frame[1])) {
    case FrameType::kTime: {
      auto parsed = std::unique_ptr<TimeFrame>(new TimeFrame());
      populateBase(*parsed, frame);
      parsed->year = frame[2];
      parsed->month = frame[3];
      parsed->day = frame[4];
      parsed->hour = frame[5];
      parsed->minute = frame[6];
      parsed->second = frame[7];
      parsed->millisecond = uint16FromLe(frame[8], frame[9]);
      return parsed;
    }
    case FrameType::kAcceleration: {
      auto parsed = std::unique_ptr<AccelerationFrame>(new AccelerationFrame());
      populateBase(*parsed, frame);
      parsed->x_mps2 = scaledInt16(frame[2], frame[3], 16.0 * kGravityMps2);
      parsed->y_mps2 = scaledInt16(frame[4], frame[5], 16.0 * kGravityMps2);
      parsed->z_mps2 = scaledInt16(frame[6], frame[7], 16.0 * kGravityMps2);
      parsed->temperature_c = static_cast<double>(int16FromLe(frame[8], frame[9])) / 100.0;
      return parsed;
    }
    case FrameType::kAngularVelocity: {
      auto parsed = std::unique_ptr<AngularVelocityFrame>(new AngularVelocityFrame());
      populateBase(*parsed, frame);
      parsed->x_radps = scaledInt16(frame[2], frame[3], 2000.0) * kPi / 180.0;
      parsed->y_radps = scaledInt16(frame[4], frame[5], 2000.0) * kPi / 180.0;
      parsed->z_radps = scaledInt16(frame[6], frame[7], 2000.0) * kPi / 180.0;
      parsed->voltage_v = static_cast<double>(uint16FromLe(frame[8], frame[9])) / 100.0;
      return parsed;
    }
    case FrameType::kAngle: {
      auto parsed = std::unique_ptr<AngleFrame>(new AngleFrame());
      populateBase(*parsed, frame);
      parsed->roll_rad = scaledInt16(frame[2], frame[3], 180.0) * kPi / 180.0;
      parsed->pitch_rad = scaledInt16(frame[4], frame[5], 180.0) * kPi / 180.0;
      parsed->yaw_rad = scaledInt16(frame[6], frame[7], 180.0) * kPi / 180.0;
      parsed->version = uint16FromLe(frame[8], frame[9]);
      return parsed;
    }
    case FrameType::kMagneticField: {
      auto parsed = std::unique_ptr<MagneticFieldFrame>(new MagneticFieldFrame());
      populateBase(*parsed, frame);
      parsed->x_raw = int16FromLe(frame[2], frame[3]);
      parsed->y_raw = int16FromLe(frame[4], frame[5]);
      parsed->z_raw = int16FromLe(frame[6], frame[7]);
      parsed->temperature_c = static_cast<double>(int16FromLe(frame[8], frame[9])) / 100.0;
      return parsed;
    }
    case FrameType::kPortStatus: {
      auto parsed = std::unique_ptr<PortStatusFrame>(new PortStatusFrame());
      populateBase(*parsed, frame);
      parsed->status = {
        uint16FromLe(frame[2], frame[3]),
        uint16FromLe(frame[4], frame[5]),
        uint16FromLe(frame[6], frame[7]),
        uint16FromLe(frame[8], frame[9])};
      return parsed;
    }
    case FrameType::kPressureAltitude: {
      auto parsed = std::unique_ptr<PressureAltitudeFrame>(new PressureAltitudeFrame());
      populateBase(*parsed, frame);
      parsed->pressure_pa = uint32FromLe(frame[2], frame[3], frame[4], frame[5]);
      parsed->altitude_cm = int32FromLe(frame[6], frame[7], frame[8], frame[9]);
      return parsed;
    }
    case FrameType::kLatitudeLongitude: {
      auto parsed = std::unique_ptr<LatitudeLongitudeFrame>(new LatitudeLongitudeFrame());
      populateBase(*parsed, frame);
      parsed->longitude_raw = int32FromLe(frame[2], frame[3], frame[4], frame[5]);
      parsed->latitude_raw = int32FromLe(frame[6], frame[7], frame[8], frame[9]);
      return parsed;
    }
    case FrameType::kGpsData: {
      auto parsed = std::unique_ptr<GpsDataFrame>(new GpsDataFrame());
      populateBase(*parsed, frame);
      parsed->gps_height_m = static_cast<double>(int16FromLe(frame[2], frame[3])) / 10.0;
      parsed->gps_yaw_deg = static_cast<double>(uint16FromLe(frame[4], frame[5])) / 100.0;
      parsed->ground_speed_kph =
        static_cast<double>(uint32FromLe(frame[6], frame[7], frame[8], frame[9])) / 1000.0;
      return parsed;
    }
    case FrameType::kQuaternion: {
      auto parsed = std::unique_ptr<QuaternionFrame>(new QuaternionFrame());
      populateBase(*parsed, frame);
      parsed->w = scaledInt16(frame[2], frame[3], 1.0);
      parsed->x = scaledInt16(frame[4], frame[5], 1.0);
      parsed->y = scaledInt16(frame[6], frame[7], 1.0);
      parsed->z = scaledInt16(frame[8], frame[9], 1.0);
      return parsed;
    }
    case FrameType::kGpsAccuracy: {
      auto parsed = std::unique_ptr<GpsAccuracyFrame>(new GpsAccuracyFrame());
      populateBase(*parsed, frame);
      parsed->satellite_count = uint16FromLe(frame[2], frame[3]);
      parsed->pdop = static_cast<double>(uint16FromLe(frame[4], frame[5])) / 100.0;
      parsed->hdop = static_cast<double>(uint16FromLe(frame[6], frame[7])) / 100.0;
      parsed->vdop = static_cast<double>(uint16FromLe(frame[8], frame[9])) / 100.0;
      return parsed;
    }
    case FrameType::kReadRegister: {
      auto parsed = std::unique_ptr<ReadRegisterFrame>(new ReadRegisterFrame());
      populateBase(*parsed, frame);
      parsed->registers = {
        uint16FromLe(frame[2], frame[3]),
        uint16FromLe(frame[4], frame[5]),
        uint16FromLe(frame[6], frame[7]),
        uint16FromLe(frame[8], frame[9])};
      return parsed;
    }
    default:
      return nullptr;
  }
}

std::unique_ptr<ImuFrameBase> ProtocolParser::feed(uint8_t byte)
{
  buffer_.push_back(byte);
  return tryParseBufferedFrame();
}

void ProtocolParser::reset()
{
  buffer_.clear();
}

std::unique_ptr<ImuFrameBase> ProtocolParser::tryParseBufferedFrame()
{
  while (!buffer_.empty() && buffer_.front() != kHeader) {
    buffer_.pop_front();
  }

  while (buffer_.size() >= kFrameSize) {
    std::array<uint8_t, kFrameSize> frame{};
    std::copy_n(buffer_.begin(), kFrameSize, frame.begin());

    auto parsed = parseFrame(frame);
    if (parsed) {
      for (std::size_t i = 0; i < kFrameSize; ++i) {
        buffer_.pop_front();
      }
      return parsed;
    }

    buffer_.pop_front();
    while (!buffer_.empty() && buffer_.front() != kHeader) {
      buffer_.pop_front();
    }
  }

  return nullptr;
}

}  // namespace yahboom_10_axis_imu
