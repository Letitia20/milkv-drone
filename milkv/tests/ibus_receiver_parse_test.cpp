#include "ibus_receiver.hpp"

#include <array>
#include <cstdint>
#include <iostream>

namespace {

void putLe16(std::array<std::uint8_t, drone::ibus::kFrameLength>& frame, std::size_t offset, std::uint16_t value) {
    frame[offset] = static_cast<std::uint8_t>(value & 0xff);
    frame[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xff);
}

std::array<std::uint8_t, drone::ibus::kFrameLength> makeFrame(
    const std::array<std::uint16_t, drone::ibus::kChannelCount>& channels) {
    std::array<std::uint8_t, drone::ibus::kFrameLength> frame {};
    frame[0] = drone::ibus::kFrameLength;
    frame[1] = drone::ibus::kCommand;

    for (std::size_t i = 0; i < channels.size(); ++i) {
        putLe16(frame, 2 + i * 2, channels[i]);
    }

    std::uint16_t checksum = drone::ibus::kChecksumInitial;
    for (std::size_t i = 0; i < drone::ibus::kChecksumBytes; ++i) {
        checksum = static_cast<std::uint16_t>(checksum - frame[i]);
    }
    putLe16(frame, drone::ibus::kChecksumBytes, checksum);
    return frame;
}

bool sameChannels(const drone::IBusChannels& actual,
                  const std::array<std::uint16_t, drone::ibus::kChannelCount>& expected) {
    for (std::size_t i = 0; i < expected.size(); ++i) {
        if (actual.ch[i] != expected[i]) {
            std::cerr << "CH" << (i + 1) << " expected " << expected[i] << " got " << actual.ch[i] << "\n";
            return false;
        }
    }
    return true;
}

}  // namespace

int main() {
    const std::array<std::uint16_t, drone::ibus::kChannelCount> expected {
        1000, 1100, 1200, 1300, 1400, 1500, 1600,
        1700, 1800, 1900, 2000, 1501, 1502, 1503,
    };

    drone::IBusChannels channels;
    const auto valid_frame = makeFrame(expected);
    if (!drone::ibus::parseFrame(valid_frame.data(), valid_frame.size(), 1234, channels) ||
        !channels.valid || channels.last_update_ms != 1234 || !sameChannels(channels, expected)) {
        std::cerr << "valid iBUS frame did not parse\n";
        return 1;
    }

    auto bad_header = valid_frame;
    bad_header[0] = 0x21;
    if (drone::ibus::parseFrame(bad_header.data(), bad_header.size(), 1234, channels)) {
        std::cerr << "bad header should fail\n";
        return 1;
    }

    auto bad_checksum = valid_frame;
    bad_checksum[10] ^= 0x01;
    if (drone::ibus::parseFrame(bad_checksum.data(), bad_checksum.size(), 1234, channels)) {
        std::cerr << "bad checksum should fail\n";
        return 1;
    }

    auto bad_channel = valid_frame;
    putLe16(bad_channel, 2 + 6 * 2, 799);
    std::uint16_t checksum = drone::ibus::kChecksumInitial;
    for (std::size_t i = 0; i < drone::ibus::kChecksumBytes; ++i) {
        checksum = static_cast<std::uint16_t>(checksum - bad_channel[i]);
    }
    putLe16(bad_channel, drone::ibus::kChecksumBytes, checksum);
    if (drone::ibus::parseFrame(bad_channel.data(), bad_channel.size(), 1234, channels)) {
        std::cerr << "out-of-range channel should fail\n";
        return 1;
    }

    if (drone::ibus::parseFrame(valid_frame.data(), valid_frame.size() - 1, 1234, channels)) {
        std::cerr << "short frame should fail\n";
        return 1;
    }

    return 0;
}
