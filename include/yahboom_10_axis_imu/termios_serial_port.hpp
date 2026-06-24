#ifndef YAHBOOM_10_AXIS_IMU_TERMIOS_SERIAL_PORT_HPP_
#define YAHBOOM_10_AXIS_IMU_TERMIOS_SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include <sys/types.h>

namespace yahboom_10_axis_imu
{

class TermiosSerialPort
{
public:
  TermiosSerialPort() = default;
  ~TermiosSerialPort();

  TermiosSerialPort(const TermiosSerialPort &) = delete;
  TermiosSerialPort & operator=(const TermiosSerialPort &) = delete;

  void open(const std::string & port, int baud_rate);
  void close();
  bool isOpen() const;
  ssize_t read(uint8_t * buffer, std::size_t length);
  void write(const uint8_t * data, std::size_t length);

private:
  int fd_{-1};
};

}  // namespace yahboom_10_axis_imu

#endif  // YAHBOOM_10_AXIS_IMU_TERMIOS_SERIAL_PORT_HPP_
