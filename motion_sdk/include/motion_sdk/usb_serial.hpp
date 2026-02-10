#pragma once

#include <string>

namespace motion_sdk {

class USBSerial {
public:
    USBSerial(const std::string& device, int baudrate);
    ~USBSerial();

    bool open();
    void close();

    bool writeLine(const std::string& line);
    bool readLine(std::string& line);

private:
    std::string device_;
    int baudrate_;
    int fd_;
};

} // namespace motion_sdk
