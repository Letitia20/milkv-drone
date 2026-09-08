#include "ibus_receiver.hpp"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>

#ifndef _WIN32
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace drone {

namespace ibus {

bool parseFrame(const std::uint8_t* frame, std::size_t length, std::uint64_t now_ms, IBusChannels& out) {
    if (frame == nullptr || length != kFrameLength) {
        return false;
    }

    if (frame[0] != kFrameLength || frame[1] != kCommand) {
        return false;
    }

    // iBUS 校验公式：
    // checksum = 0xffff - sum(frame[0..29])
    std::uint16_t checksum = kChecksumInitial;
    for (std::size_t i = 0; i < kChecksumBytes; ++i) {
        checksum = static_cast<std::uint16_t>(checksum - frame[i]);
    }

    const std::uint16_t expected = static_cast<std::uint16_t>(frame[30]) |
                                   static_cast<std::uint16_t>(frame[31] << 8);
    if (checksum != expected) {
        return false;
    }

    // 通道值为小端 16 位：ch = low | (high << 8)，单位为 us。
    IBusChannels parsed;
    for (std::size_t ch = 0; ch < parsed.ch.size(); ++ch) {
        const std::size_t offset = 2 + ch * 2;
        parsed.ch[ch] = static_cast<std::uint16_t>(frame[offset]) |
                        static_cast<std::uint16_t>(frame[offset + 1] << 8);
        if (parsed.ch[ch] < kMinChannelValue || parsed.ch[ch] > kMaxChannelValue) {
            return false;
        }
    }

    parsed.valid = true;
    parsed.last_update_ms = now_ms;
    out = parsed;
    return true;
}

std::uint64_t monotonicMs() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

}  // namespace ibus

IBusReceiver::~IBusReceiver() {
    close();
}

const char* IBusReceiver::lastError() const {
    return last_error_.data();
}

void IBusReceiver::setError(const char* prefix) {
    std::snprintf(last_error_.data(), last_error_.size(), "%s: %s", prefix, std::strerror(errno));
}

bool IBusReceiver::ingestByte(std::uint8_t byte, IBusChannels& out) {
    if (frame_pos_ == 0) {
        if (byte == ibus::kFrameLength) {
            frame_[frame_pos_++] = byte;
        }
        return false;
    }

    if (frame_pos_ == 1 && byte != ibus::kCommand) {
        frame_pos_ = (byte == ibus::kFrameLength) ? 1 : 0;
        frame_[0] = ibus::kFrameLength;
        return false;
    }

    frame_[frame_pos_++] = byte;
    if (frame_pos_ < frame_.size()) {
        return false;
    }

    frame_pos_ = 0;
    return ibus::parseFrame(frame_.data(), frame_.size(), ibus::monotonicMs(), out);
}

#ifndef _WIN32
namespace {

bool configureIbusSerial(int fd) {
    termios tty {};
    if (::tcgetattr(fd, &tty) != 0) {
        return false;
    }

    ::cfmakeraw(&tty);
    ::cfsetispeed(&tty, B115200);
    ::cfsetospeed(&tty, B115200);

    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        return false;
    }

    ::tcflush(fd, TCIFLUSH);
    return true;
}

}  // namespace

bool IBusReceiver::open(const char* dev) {
    close();
    frame_pos_ = 0;
    last_error_[0] = '\0';

    fd_ = ::open(dev, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        setError("open iBUS serial");
        return false;
    }

    if (!configureIbusSerial(fd_)) {
        setError("configure iBUS serial");
        close();
        return false;
    }

    return true;
}

bool IBusReceiver::readFrame(IBusChannels& out) {
    if (fd_ < 0) {
        return false;
    }

    std::uint8_t byte = 0;
    for (;;) {
        const ssize_t n = ::read(fd_, &byte, 1);
        if (n == 1) {
            if (ingestByte(byte, out)) {
                return true;
            }
            continue;
        }
        if (n == 0 || (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
            return false;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        setError("read iBUS serial");
        return false;
    }
}

void IBusReceiver::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

#else

bool IBusReceiver::open(const char*) {
    std::snprintf(last_error_.data(), last_error_.size(), "iBUS serial is unavailable on this host");
    return false;
}

bool IBusReceiver::readFrame(IBusChannels&) {
    return false;
}

void IBusReceiver::close() {
    fd_ = -1;
}

#endif

}  // namespace drone
