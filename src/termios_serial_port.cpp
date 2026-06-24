#include "yahboom_10_axis_imu/termios_serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace yahboom_10_axis_imu
{
namespace
{

speed_t baudToTermios(int baud_rate)
{
  switch (baud_rate) {
    case 1200:
      return B1200;
    case 2400:
      return B2400;
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      throw std::invalid_argument("unsupported baud_rate: " + std::to_string(baud_rate));
  }
}

std::string errnoMessage(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

}  // namespace

TermiosSerialPort::~TermiosSerialPort()
{
  close();
}

void TermiosSerialPort::open(const std::string & port, int baud_rate)
{
  close();

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error(errnoMessage("failed to open serial port " + port));
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close();
    throw std::runtime_error(errnoMessage("failed to read serial port attributes"));
  }

  cfmakeraw(&tty);

  const speed_t speed = baudToTermios(baud_rate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  tty.c_cflag |= CS8;
  tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
  tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    throw std::runtime_error(errnoMessage("failed to configure serial port"));
  }

  tcflush(fd_, TCIOFLUSH);
}

void TermiosSerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool TermiosSerialPort::isOpen() const
{
  return fd_ >= 0;
}

ssize_t TermiosSerialPort::read(uint8_t * buffer, std::size_t length)
{
  if (fd_ < 0) {
    throw std::runtime_error("serial port is not open");
  }
  return ::read(fd_, buffer, length);
}

void TermiosSerialPort::write(const uint8_t * data, std::size_t length)
{
  if (fd_ < 0) {
    throw std::runtime_error("serial port is not open");
  }

  std::size_t bytes_written = 0;
  while (bytes_written < length) {
    const ssize_t result = ::write(fd_, data + bytes_written, length - bytes_written);
    if (result < 0) {
      throw std::runtime_error(errnoMessage("serial write failed"));
    }
    if (result == 0) {
      throw std::runtime_error("serial write wrote zero bytes");
    }
    bytes_written += static_cast<std::size_t>(result);
  }

  if (tcdrain(fd_) != 0) {
    throw std::runtime_error(errnoMessage("failed to drain serial write"));
  }
}

}  // namespace yahboom_10_axis_imu
