#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float64.hpp"

#include "yahboom_10_axis_imu/imu_packet.hpp"
#include "yahboom_10_axis_imu/protocol.hpp"
#include "yahboom_10_axis_imu/termios_serial_port.hpp"
#include "yahboom_10_axis_imu/write_protocol.hpp"

namespace yahboom_10_axis_imu
{
namespace
{

void markUnknown(std::array<double, 9> & covariance)
{
  covariance.fill(0.0);
}

void setDiagonalCovariance(std::array<double, 9> & covariance, double variance)
{
  covariance.fill(0.0);
  covariance[0] = variance;
  covariance[4] = variance;
  covariance[8] = variance;
}

bool normalize(Quaternion & quaternion)
{
  const double norm = std::sqrt(
    quaternion.w * quaternion.w +
    quaternion.x * quaternion.x +
    quaternion.y * quaternion.y +
    quaternion.z * quaternion.z);

  if (norm <= 1e-9) {
    return false;
  }

  quaternion.w /= norm;
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  return true;
}

Quaternion quaternionFromEuler(double roll, double pitch, double yaw)
{
  const double half_roll = roll * 0.5;
  const double half_pitch = pitch * 0.5;
  const double half_yaw = yaw * 0.5;

  const double cr = std::cos(half_roll);
  const double sr = std::sin(half_roll);
  const double cp = std::cos(half_pitch);
  const double sp = std::sin(half_pitch);
  const double cy = std::cos(half_yaw);
  const double sy = std::sin(half_yaw);

  Quaternion quaternion;
  quaternion.w = cr * cp * cy + sr * sp * sy;
  quaternion.x = sr * cp * cy - cr * sp * sy;
  quaternion.y = cr * sp * cy + sr * cp * sy;
  quaternion.z = cr * cp * sy - sr * sp * cy;
  return quaternion;
}

const char * frameTypeName(uint8_t type)
{
  switch (static_cast<FrameType>(type)) {
    case FrameType::kTime:
      return "time";
    case FrameType::kAcceleration:
      return "acceleration";
    case FrameType::kAngularVelocity:
      return "angular_velocity";
    case FrameType::kAngle:
      return "angle";
    case FrameType::kMagneticField:
      return "magnetic_field";
    case FrameType::kPortStatus:
      return "port_status";
    case FrameType::kPressureAltitude:
      return "pressure_altitude";
    case FrameType::kLatitudeLongitude:
      return "latitude_longitude";
    case FrameType::kGpsData:
      return "gps_data";
    case FrameType::kQuaternion:
      return "quaternion";
    case FrameType::kGpsAccuracy:
      return "gps_accuracy";
    case FrameType::kReadRegister:
      return "read_register";
    default:
      return "unknown";
  }
}

}  // namespace

class Yahboom10AxisImuNode : public rclcpp::Node
{
public:
  Yahboom10AxisImuNode()
  : Node("yahboom_10_axis_imu_node")
  {
    port_ = declare_parameter<std::string>("port", "/dev/imu_usb");
    baud_rate_ = declare_parameter<int>("baud_rate", 9600);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    use_euler_orientation_fallback_ =
      declare_parameter<bool>("use_euler_orientation_fallback", true);
    output_rate_hz_ = declare_parameter<double>("output_rate_hz", kDefaultOutputRateHz);
    save_configuration_ = declare_parameter<bool>("save_configuration", false);
    linear_acceleration_rms_mps2_ =
      declare_parameter<double>("linear_acceleration_rms_mps2", 9.81e-3);
    angular_velocity_rms_radps_ =
      declare_parameter<double>("angular_velocity_rms_radps", 0.00122111111);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());
    mag_pub_ =
      create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", rclcpp::SensorDataQoS());
    pressure_pub_ =
      create_publisher<sensor_msgs::msg::FluidPressure>("imu/pressure", rclcpp::SensorDataQoS());
    altitude_pub_ = create_publisher<std_msgs::msg::Float64>("imu/altitude", 10);
    temperature_pub_ = create_publisher<std_msgs::msg::Float64>("imu/temperature", 10);

    startReader();
  }

  ~Yahboom10AxisImuNode() override
  {
    stopReader();
  }

private:
  void startReader()
  {
    running_.store(true);
    reader_thread_ = std::thread([this]() { readLoop(); });
  }

  void stopReader()
  {
    running_.store(false);
    serial_.close();
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
  }

  void readLoop()
  {
    try {
      serial_.open(port_, baud_rate_);
      RCLCPP_INFO(get_logger(), "Opened Yahboom 10-axis IMU on %s at %d baud",
        port_.c_str(), baud_rate_);
      configureDevice();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Unable to initialize Yahboom 10-axis IMU serial port: %s",
        error.what());
      running_.store(false);
      return;
    }

    std::array<uint8_t, 256> read_buffer{};
    while (rclcpp::ok() && running_.load()) {
      ssize_t bytes_read = 0;
      try {
        bytes_read = serial_.read(read_buffer.data(), read_buffer.size());
      } catch (const std::exception & error) {
        if (running_.load()) {
          RCLCPP_ERROR(get_logger(), "Serial read failed: %s", error.what());
        }
        break;
      }

      if (bytes_read < 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Serial read returned an error");
        continue;
      }

      for (ssize_t i = 0; i < bytes_read; ++i) {
        auto frame = parser_.feed(read_buffer[static_cast<std::size_t>(i)]);
        if (frame) {
          RCLCPP_INFO(
            get_logger(), "Acquired IMU frame: type=0x%02X (%s)",
            frame->type, frameTypeName(frame->type));
          handleFrame(*frame);
        }
      }
    }
  }

  void handleFrame(const ImuFrameBase & frame)
  {
    if (const auto * acceleration = dynamic_cast<const AccelerationFrame *>(&frame)) {
      handleAcceleration(*acceleration);
      return;
    }

    if (const auto * angular_velocity = dynamic_cast<const AngularVelocityFrame *>(&frame)) {
      handleAngularVelocity(*angular_velocity);
      return;
    }

    if (const auto * quaternion = dynamic_cast<const QuaternionFrame *>(&frame)) {
      handleQuaternion(*quaternion);
      return;
    }

    if (const auto * angle = dynamic_cast<const AngleFrame *>(&frame)) {
      handleAngle(*angle);
      return;
    }

    if (const auto * magnetic_field = dynamic_cast<const MagneticFieldFrame *>(&frame)) {
      publishMagneticField(*magnetic_field);
      publishTemperature(magnetic_field->temperature_c);
      return;
    }

    if (const auto * pressure_altitude = dynamic_cast<const PressureAltitudeFrame *>(&frame)) {
      publishPressureAltitude(*pressure_altitude);
      return;
    }
  }

  void handleAcceleration(const AccelerationFrame & frame)
  {
    {
      std::lock_guard<std::mutex> lock(assembler_mutex_);
      assembler_.update(frame);
      logReadinessMaskLocked("acceleration");
    }
    publishTemperature(frame.temperature_c);
    publishImuIfReady();
  }

  void handleAngularVelocity(const AngularVelocityFrame & frame)
  {
    {
      std::lock_guard<std::mutex> lock(assembler_mutex_);
      assembler_.update(frame);
      logReadinessMaskLocked("angular_velocity");
    }
    publishImuIfReady();
  }

  void handleQuaternion(const QuaternionFrame & frame)
  {
    Quaternion quaternion{frame.w, frame.x, frame.y, frame.z};
    if (!normalize(quaternion)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid zero quaternion");
      return;
    }

    QuaternionFrame normalized_frame = frame;
    normalized_frame.w = quaternion.w;
    normalized_frame.x = quaternion.x;
    normalized_frame.y = quaternion.y;
    normalized_frame.z = quaternion.z;

    {
      std::lock_guard<std::mutex> lock(assembler_mutex_);
      assembler_.update(normalized_frame);
      logReadinessMaskLocked("quaternion");
    }
    publishImuIfReady();
  }

  void handleAngle(const AngleFrame & frame)
  {
    if (!use_euler_orientation_fallback_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Received angle frame 0x53, but Euler orientation fallback is disabled");
      return;
    }

    Quaternion quaternion = quaternionFromEuler(frame.roll_rad, frame.pitch_rad, frame.yaw_rad);
    if (!normalize(quaternion)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring invalid Euler quaternion");
      return;
    }

    QuaternionFrame fallback_frame;
    fallback_frame.w = quaternion.w;
    fallback_frame.x = quaternion.x;
    fallback_frame.y = quaternion.y;
    fallback_frame.z = quaternion.z;

    {
      std::lock_guard<std::mutex> lock(assembler_mutex_);
      assembler_.update(fallback_frame);
      logReadinessMaskLocked("angle_fallback_orientation");
    }
    publishImuIfReady();
  }

  void publishImuIfReady()
  {
    ImuPacket packet;
    {
      std::lock_guard<std::mutex> lock(assembler_mutex_);
      if (!assembler_.ready()) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting to publish imu/data: readiness_mask=0b%u%u%u "
          "(accel=%s, gyro=%s, orientation=%s)",
          (assembler_.readyMask() & kOrientationReady) ? 1 : 0,
          (assembler_.readyMask() & kGyroReady) ? 1 : 0,
          (assembler_.readyMask() & kAccelReady) ? 1 : 0,
          (assembler_.readyMask() & kAccelReady) ? "ready" : "missing",
          (assembler_.readyMask() & kGyroReady) ? "ready" : "missing",
          (assembler_.readyMask() & kOrientationReady) ? "ready" : "missing");
        return;
      }
      packet = assembler_.consume();
    }

    sensor_msgs::msg::Imu message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;

    message.linear_acceleration.x = packet.linear_acceleration.x;
    message.linear_acceleration.y = packet.linear_acceleration.y;
    message.linear_acceleration.z = packet.linear_acceleration.z;

    message.angular_velocity.x = packet.angular_velocity.x;
    message.angular_velocity.y = packet.angular_velocity.y;
    message.angular_velocity.z = packet.angular_velocity.z;

    message.orientation.w = packet.orientation.w;
    message.orientation.x = packet.orientation.x;
    message.orientation.y = packet.orientation.y;
    message.orientation.z = packet.orientation.z;

    markUnknown(message.orientation_covariance);
    setDiagonalCovariance(
      message.angular_velocity_covariance,
      angular_velocity_rms_radps_ * angular_velocity_rms_radps_);
    setDiagonalCovariance(
      message.linear_acceleration_covariance,
      linear_acceleration_rms_mps2_ * linear_acceleration_rms_mps2_);

    imu_pub_->publish(message);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Published imu/data: accel=[%.3f, %.3f, %.3f] m/s^2, gyro=[%.3f, %.3f, %.3f] rad/s, "
      "quat=[w=%.4f, x=%.4f, y=%.4f, z=%.4f]",
      message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z,
      message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z,
      message.orientation.w, message.orientation.x, message.orientation.y, message.orientation.z);
  }

  void publishMagneticField(const MagneticFieldFrame & frame)
  {
    sensor_msgs::msg::MagneticField message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;

    message.magnetic_field.x = static_cast<double>(frame.x_raw);
    message.magnetic_field.y = static_cast<double>(frame.y_raw);
    message.magnetic_field.z = static_cast<double>(frame.z_raw);
    markUnknown(message.magnetic_field_covariance);

    mag_pub_->publish(message);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Published imu/mag: magnetic_field_raw=[%.0f, %.0f, %.0f]",
      message.magnetic_field.x, message.magnetic_field.y, message.magnetic_field.z);
  }

  void publishPressureAltitude(const PressureAltitudeFrame & frame)
  {
    sensor_msgs::msg::FluidPressure pressure;
    pressure.header.stamp = now();
    pressure.header.frame_id = frame_id_;
    pressure.fluid_pressure = static_cast<double>(frame.pressure_pa);
    pressure.variance = 0.0;
    pressure_pub_->publish(pressure);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Published imu/pressure: %.2f Pa", pressure.fluid_pressure);

    std_msgs::msg::Float64 altitude;
    altitude.data = static_cast<double>(frame.altitude_cm) / 100.0;
    altitude_pub_->publish(altitude);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Published imu/altitude: %.2f m", altitude.data);
  }

  void publishTemperature(double temperature_c)
  {
    std_msgs::msg::Float64 temperature;
    temperature.data = temperature_c;
    temperature_pub_->publish(temperature);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Published imu/temperature: %.2f C", temperature.data);
  }

  void configureDevice()
  {
    bool used_rate_fallback = false;
    const uint16_t rrate_value = outputRateHzToRRateValue(output_rate_hz_, &used_rate_fallback);
    const double applied_output_rate_hz =
      used_rate_fallback ? kDefaultOutputRateHz : output_rate_hz_;

    if (used_rate_fallback) {
      RCLCPP_ERROR(
        get_logger(),
        "Unsupported output_rate_hz %.3f. Falling back to %.1f Hz",
        output_rate_hz_, kDefaultOutputRateHz);
    }

    writeRegister("KEY unlock", kRegisterKey, kUnlockValue, "unlock writes");
    writeRegister("RRATE", kRegisterRRate, rrate_value, "%.1f Hz output rate",
      applied_output_rate_hz);
    writeRegister("BANDWIDTH", kRegisterBandwidth, kHighestBandwidthValue,
      "256 Hz bandwidth");

    if (save_configuration_) {
      writeRegister("SAVE", kRegisterSave, kSaveValue, "persist configuration");
    } else {
      RCLCPP_INFO(get_logger(), "Skipping SAVE write because save_configuration is false");
    }
  }

  void writeRegister(
    const char * name,
    uint8_t register_address,
    uint16_t value,
    const char * meaning)
  {
    const auto frame = makeWriteFrame(register_address, value);
    serial_.write(frame.data(), frame.size());
    RCLCPP_INFO(
      get_logger(),
      "Wrote %s register 0x%02X value 0x%04X (%s): %02X %02X %02X %02X %02X",
      name, register_address, value, meaning,
      frame[0], frame[1], frame[2], frame[3], frame[4]);
  }

  template<typename... Args>
  void writeRegister(
    const char * name,
    uint8_t register_address,
    uint16_t value,
    const char * meaning_format,
    Args... args)
  {
    char meaning[128];
    std::snprintf(meaning, sizeof(meaning), meaning_format, args...);
    writeRegister(name, register_address, value, meaning);
  }

  void logReadinessMaskLocked(const char * updated_field)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Updated IMU readiness from %s: mask=0b%u%u%u",
      updated_field,
      (assembler_.readyMask() & kOrientationReady) ? 1 : 0,
      (assembler_.readyMask() & kGyroReady) ? 1 : 0,
      (assembler_.readyMask() & kAccelReady) ? 1 : 0);
  }

  std::string port_;
  int baud_rate_{9600};
  std::string frame_id_;
  bool use_euler_orientation_fallback_{true};
  double output_rate_hz_{kDefaultOutputRateHz};
  bool save_configuration_{false};
  double linear_acceleration_rms_mps2_{9.81e-3};
  double angular_velocity_rms_radps_{0.00122111111};

  TermiosSerialPort serial_;
  ProtocolParser parser_;
  ImuPacketAssembler assembler_;
  std::mutex assembler_mutex_;
  std::atomic<bool> running_{false};
  std::thread reader_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr altitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temperature_pub_;
};

}  // namespace yahboom_10_axis_imu

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<yahboom_10_axis_imu::Yahboom10AxisImuNode>());
  rclcpp::shutdown();
  return 0;
}
