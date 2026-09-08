#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace drone {

namespace ibus {

// iBUS 帧格式：32 字节，起始长度 0x20，命令字 0x40，后面是 14 路通道和校验。
constexpr std::size_t kFrameLength = 32;
constexpr std::uint8_t kCommand = 0x40;
constexpr std::size_t kChecksumBytes = 30;
constexpr std::uint16_t kChecksumInitial = 0xffff;
constexpr std::size_t kChannelCount = 14;
constexpr std::uint16_t kMinChannelValue = 800;
constexpr std::uint16_t kMaxChannelValue = 2200;

}  // namespace ibus

struct IBusChannels {
    std::array<std::uint16_t, ibus::kChannelCount> ch {};
    bool valid {false};
    std::uint64_t last_update_ms {0};
};

namespace ibus {

bool parseFrame(const std::uint8_t* frame, std::size_t length, std::uint64_t now_ms, IBusChannels& out);
std::uint64_t monotonicMs();

}  // namespace ibus

class IBusReceiver {
public:
    IBusReceiver() = default;
    ~IBusReceiver();

    IBusReceiver(const IBusReceiver&) = delete;
    IBusReceiver& operator=(const IBusReceiver&) = delete;

    bool open(const char* dev = "/dev/ttyS1");
    bool readFrame(IBusChannels& out);
    void close();
    const char* lastError() const;

private:
    int fd_ {-1};
    std::array<std::uint8_t, ibus::kFrameLength> frame_ {};
    std::size_t frame_pos_ {0};
    std::array<char, 160> last_error_ {};

    void setError(const char* prefix);
    bool ingestByte(std::uint8_t byte, IBusChannels& out);
};

}  // namespace drone
