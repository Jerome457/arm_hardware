#include "motion_sdk/usb_serial.hpp"
#include "motion_sdk/servo.hpp"

#include <iostream>

using namespace motion_sdk;

int main() {
    USBSerial serial("/dev/cu.usbmodem8D6F436B54541", 9600);

    if (!serial.open()) {
        std::cerr << "Failed to open serial\n";
        return 1;
    }

    Servo servo(serial, 0);

    if (!servo.ping()) {
        std::cerr << "PING failed\n";
        return 1;
    }

    servo.setPosition(0.0f);

    float pos = 0.0f;
    if (servo.getPosition(pos)) {
        std::cout << "Position: " << pos << " deg\n";
    } else {
        std::cerr << "GET_POS failed\n";
    }

    return 0;
}
