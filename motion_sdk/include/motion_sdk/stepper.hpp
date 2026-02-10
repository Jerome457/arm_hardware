#pragma once

#include "motion_sdk/usb_serial.hpp"
#include <iostream>

namespace motion_sdk {

class Stepper {
public:
    Stepper(USBSerial& serial);

    bool ping();
    bool setPosition(float angle_deg);
    bool getPosition(float& angle_deg);

private:
    USBSerial& serial_;
};

} // namespace motion_sdk
