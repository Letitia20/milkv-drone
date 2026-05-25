#pragma once

#include <string>
#include <vector>

namespace drone {

class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& device, int baudrate);
    void close();

    bool isOpen() const;
    std::vector<std::string> readLines();
    bool writeLine(const std::string& line);

    const std::string& lastError() const;

private:
    int fd_;
    std::string rx_buffer_;
    std::string last_error_;
};

}  // namespace drone
