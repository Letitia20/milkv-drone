#include "serial.hpp"

#include <chrono>
#include <thread>

#ifdef __linux__
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace drone {

SerialPort::SerialPort() : fd_(-1) {}

SerialPort::~SerialPort() {
    close();
}

const std::string& SerialPort::lastError() const {
    return last_error_;
}

#ifdef __linux__
namespace {

speed_t baudToTermios(int baudrate) {
    switch (baudrate) {
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
#ifdef B460800
        case 460800:
            return B460800;
#endif
#ifdef B921600
        case 921600:
            return B921600;
#endif
        default:
            return 0;
    }
}

std::string errnoText(const std::string& prefix) {
    return prefix + ": " + std::strerror(errno);
}

}  // namespace

bool SerialPort::open(const std::string& device, int baudrate) {
    close();

    const speed_t speed = baudToTermios(baudrate);
    if (speed == 0) {
        last_error_ = "Unsupported baudrate: " + std::to_string(baudrate);
        return false;
    }

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        last_error_ = errnoText("Failed to open serial device " + device);
        return false;
    }

    termios tty {};
    if (tcgetattr(fd_, &tty) != 0) {
        last_error_ = errnoText("tcgetattr failed");
        close();
        return false;
    }

    tty.c_iflag &= static_cast<tcflag_t>(
        ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON));
    tty.c_oflag &= static_cast<tcflag_t>(~OPOST);
    tty.c_lflag &= static_cast<tcflag_t>(~(ECHO | ECHONL | ICANON | ISIG | IEXTEN));
    tty.c_cflag &= static_cast<tcflag_t>(~(CSIZE | PARENB | CSTOPB));
    tty.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
#ifdef CRTSCTS
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
        last_error_ = errnoText("Failed to set serial baudrate");
        close();
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        last_error_ = errnoText("tcsetattr failed");
        close();
        return false;
    }

    last_error_.clear();
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    rx_buffer_.clear();
}

bool SerialPort::isOpen() const {
    return fd_ >= 0;
}

std::vector<std::string> SerialPort::readLines() {
    std::vector<std::string> lines;
    if (!isOpen()) {
        return lines;
    }

    char buffer[256];
    for (;;) {
        const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
        if (n > 0) {
            rx_buffer_.append(buffer, buffer + n);
            if (rx_buffer_.size() > 4096) {
                rx_buffer_.erase(0, rx_buffer_.size() - 1024);
                last_error_ = "Serial receive buffer overflow; old data dropped";
            }
            continue;
        }

        if (n == 0) {
            break;
        }

        if (errno == EINTR) {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
        }

        last_error_ = errnoText("Serial read failed");
        break;
    }

    for (;;) {
        const auto pos = rx_buffer_.find('\n');
        if (pos == std::string::npos) {
            break;
        }

        std::string line = rx_buffer_.substr(0, pos);
        rx_buffer_.erase(0, pos + 1);

        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (!line.empty()) {
            lines.push_back(line);
        }
    }

    return lines;
}

bool SerialPort::writeLine(const std::string& line) {
    if (!isOpen()) {
        last_error_ = "Serial port is not open";
        return false;
    }

    std::string out = line;
    if (out.empty() || out.back() != '\n') {
        out.push_back('\n');
    }

    std::size_t written = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
    while (written < out.size()) {
        const ssize_t n = ::write(fd_, out.data() + written, out.size() - written);
        if (n > 0) {
            written += static_cast<std::size_t>(n);
            continue;
        }

        if (n < 0 && errno == EINTR) {
            continue;
        }

        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (std::chrono::steady_clock::now() >= deadline) {
                last_error_ = "Serial write timed out";
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        last_error_ = errnoText("Serial write failed");
        return false;
    }

    return true;
}

#else

bool SerialPort::open(const std::string& device, int baudrate) {
    (void)device;
    (void)baudrate;
    close();
    last_error_ = "SerialPort termios backend requires Linux";
    return false;
}

void SerialPort::close() {
    fd_ = -1;
    rx_buffer_.clear();
}

bool SerialPort::isOpen() const {
    return false;
}

std::vector<std::string> SerialPort::readLines() {
    return {};
}

bool SerialPort::writeLine(const std::string& line) {
    (void)line;
    last_error_ = "SerialPort termios backend requires Linux";
    return false;
}

#endif

}  // namespace drone
