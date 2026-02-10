#include "motion_sdk/stepper.hpp"

#include <sstream>

namespace motion_sdk {

Stepper::Stepper(USBSerial& serial)
    : serial_(serial) {}

bool Stepper::ping() {
    if (!serial_.writeLine("PING"))
        return false;

    std::string resp;
    if (!serial_.readLine(resp))
        return false;

    return resp == "OK";
}

bool Stepper::setPosition(float angle_deg) {
    std::ostringstream cmd;
    cmd << "SET_POS " << angle_deg;
    // std::cout << "TX: " << cmd.str() << std::endl;
    if (!serial_.writeLine(cmd.str()))
        return false;

    std::string resp;
    if (!serial_.readLine(resp))
        return false;

    return resp == "OK";
}

bool Stepper::getPosition(float& angle_deg) {
    if (!serial_.writeLine("GET_POS"))
        return false;

    std::string resp;
    if (!serial_.readLine(resp))
        return false;

    try {
        angle_deg = std::stof(resp);
    } catch (...) {
        return false;
    }

    return true;
}

} // namespace motion_sdk
