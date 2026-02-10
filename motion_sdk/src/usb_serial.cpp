#include "motion_sdk/usb_serial.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

namespace motion_sdk {

USBSerial::USBSerial(const std::string& device, int baudrate)
    : device_(device), baudrate_(baudrate), fd_(-1) {}

USBSerial::~USBSerial() {
    close();
}

bool USBSerial::open() {
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0)
        return false;

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0)
        return false;

    cfmakeraw(&tty);

    speed_t speed = B115200;
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;

    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;

    return tcsetattr(fd_, TCSANOW, &tty) == 0;
}

void USBSerial::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool USBSerial::writeLine(const std::string& line) {
    if (fd_ < 0)
        return false;

    std::string msg = line + "\n";
    ssize_t written = ::write(fd_, msg.c_str(), msg.size());
    return written == static_cast<ssize_t>(msg.size());
}

bool USBSerial::readLine(std::string& line) {
    if (fd_ < 0)
        return false;

    line.clear();
    char c;

    while (true) {
        ssize_t n = ::read(fd_, &c, 1);
        if (n <= 0)
            return false;

        if (c == '\n')
            break;

        if (c != '\r')
            line += c;
    }
    return true;
}

} // namespace motion_sdk
