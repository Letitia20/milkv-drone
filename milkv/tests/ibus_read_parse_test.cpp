#define IBUS_READ_UNIT_TEST
#include "../src/ibus_read.cpp"

#include <array>
#include <cstdint>
#include <iostream>

namespace {

void putLe16(std::array<std::uint8_t, ibus::kFrameLength>& frame, std::size_t offset, std::uint16_t value) {
    frame[offset] = static_cast<std::uint8_t>(value & 0xff);
    frame[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xff);
}

std::array<std::uint8_t, ibus::kFrameLength> makeFrame(const std::array<std::uint16_t, ibus::kChannelCount>& channels) {
    std::array<std::uint8_t, ibus::kFrameLength> frame {};
    frame[0] = ibus::kFrameLength;
    frame[1] = ibus::kCommand;

    for (std::size_t i = 0; i < channels.size(); ++i) {
        putLe16(frame, 2 + i * 2, channels[i]);
    }

    std::uint16_t checksum = ibus::kChecksumInitial;
    for (std::size_t i = 0; i < ibus::kChecksumBytes; ++i) {
        checksum = static_cast<std::uint16_t>(checksum - frame[i]);
    }
    putLe16(frame, ibus::kChecksumBytes, checksum);
    return frame;
}

bool equalChannels(const std::array<std::uint16_t, ibus::kChannelCount>& actual,
                   const std::array<std::uint16_t, ibus::kChannelCount>& expected) {
    for (std::size_t i = 0; i < actual.size(); ++i) {
        if (actual[i] != expected[i]) {
            std::cerr << "CH" << (i + 1) << " expected " << expected[i] << " got " << actual[i] << "\n";
            return false;
        }
    }
    return true;
}

}  // namespace

int main() {
    const std::array<std::uint16_t, ibus::kChannelCount> expected {
        1000, 1100, 1200, 1300, 1400, 1500, 1600,
        1700, 1800, 1900, 2000, 1501, 1502, 1503,
    };

    ibus::Channels channels {};
    const auto valid_frame = makeFrame(expected);
    if (!ibus::parseFrame(valid_frame.data(), valid_frame.size(), channels) || !equalChannels(channels, expected)) {
        std::cerr << "valid iBUS frame did not parse\n";
        return 1;
    }

    auto bad_header = valid_frame;
    bad_header[0] = 0x21;
    if (ibus::parseFrame(bad_header.data(), bad_header.size(), channels)) {
        std::cerr << "bad header should fail\n";
        return 1;
    }

    auto bad_checksum = valid_frame;
    bad_checksum[10] ^= 0x01;
    if (ibus::parseFrame(bad_checksum.data(), bad_checksum.size(), channels)) {
        std::cerr << "bad checksum should fail\n";
        return 1;
    }

    if (ibus::parseFrame(valid_frame.data(), valid_frame.size() - 1, channels)) {
        std::cerr << "short frame should fail\n";
        return 1;
    }

    return 0;
}
