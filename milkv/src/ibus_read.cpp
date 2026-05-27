#include <array>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

#ifndef IBUS_READ_UNIT_TEST
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace ibus {

constexpr std::size_t kFrameLength = 32;
constexpr std::uint8_t kCommand = 0x40;
constexpr std::size_t kChecksumBytes = 30;
constexpr std::uint16_t kChecksumInitial = 0xffff;
constexpr std::size_t kChannelCount = 14;

using Channels = std::array<std::uint16_t, kChannelCount>;

bool parseFrame(const std::uint8_t* frame, std::size_t length, Channels& channels) {
    if (frame == nullptr || length != kFrameLength) {
        return false;
    }

    if (frame[0] != kFrameLength || frame[1] != kCommand) {
        return false;
    }

    std::uint16_t checksum = kChecksumInitial;
    for (std::size_t i = 0; i < kChecksumBytes; ++i) {
        checksum = static_cast<std::uint16_t>(checksum - frame[i]);
    }

    const std::uint16_t expected = static_cast<std::uint16_t>(frame[30]) |
                                   static_cast<std::uint16_t>(frame[31] << 8);
    if (checksum != expected) {
        return false;
    }

    for (std::size_t ch = 0; ch < channels.size(); ++ch) {
        const std::size_t offset = 2 + ch * 2;
        channels[ch] = static_cast<std::uint16_t>(frame[offset]) |
                       static_cast<std::uint16_t>(frame[offset + 1] << 8);
    }

    return true;
}

}  // namespace ibus

#ifndef IBUS_READ_UNIT_TEST
namespace {

volatile std::sig_atomic_t g_running = 1;

void handleSignal(int) {
    g_running = 0;
}

bool configureSerial(int fd) {
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
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        return false;
    }

    ::tcflush(fd, TCIFLUSH);
    return true;
}

bool readByte(int fd, std::uint8_t& value) {
    for (;;) {
        const ssize_t n = ::read(fd, &value, 1);
        if (n == 1) {
            return true;
        }
        if (n < 0 && errno == EINTR && g_running) {
            continue;
        }
        return false;
    }
}

bool readCandidateFrame(int fd, std::array<std::uint8_t, ibus::kFrameLength>& frame) {
    std::uint8_t byte = 0;

    while (g_running) {
        if (!readByte(fd, byte)) {
            return false;
        }
        if (byte != ibus::kFrameLength) {
            continue;
        }

        if (!readByte(fd, byte)) {
            return false;
        }
        if (byte != ibus::kCommand) {
            continue;
        }

        frame[0] = ibus::kFrameLength;
        frame[1] = ibus::kCommand;
        for (std::size_t i = 2; i < frame.size(); ++i) {
            if (!readByte(fd, frame[i])) {
                return false;
            }
        }
        return true;
    }

    return false;
}

void printChannels(const ibus::Channels& channels) {
    for (std::size_t i = 0; i < channels.size(); ++i) {
        if (i > 0) {
            std::cout << ' ';
        }
        std::cout << "CH" << (i + 1) << '=' << channels[i];
    }
    std::cout << '\n';
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string device = (argc >= 2) ? argv[1] : "/dev/ttyS0";

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    const int fd = ::open(device.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Failed to open " << device << ": " << std::strerror(errno) << "\n";
        return 1;
    }

    if (!configureSerial(fd)) {
        std::cerr << "Failed to configure " << device << ": " << std::strerror(errno) << "\n";
        ::close(fd);
        return 1;
    }

    std::cerr << "Reading FlySky iBUS from " << device << " at 115200 8N1 raw\n";

    std::array<std::uint8_t, ibus::kFrameLength> frame {};
    ibus::Channels channels {};
    while (g_running && readCandidateFrame(fd, frame)) {
        if (ibus::parseFrame(frame.data(), frame.size(), channels)) {
            printChannels(channels);
        }
    }

    ::close(fd);
    return 0;
}
#endif
