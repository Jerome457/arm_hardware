#include "motion_sdk/servo.hpp"

#include <sstream>

namespace motion_sdk {

Servo::Servo(USBSerial& serial, int id)
    : serial_(serial), id_(id) {}

bool Servo::ping() {
    std::ostringstream cmd;
    cmd << "SET_POS " << id_ << " ";
    if (!serial_.writeLine(cmd.str()))
        return false;

    std::string resp;
    if (!serial_.readLine(resp))
        return false;

    return resp == "OK";
}

bool Servo::setPosition(float angle_deg) {
    std::ostringstream cmd;
    cmd << "SET_POS " << id_ << " " << angle_deg;

    //std::cout<<cmd.str()<<'\n';

    if (!serial_.writeLine(cmd.str()))
        return false;

    std::string resp;
    if (!serial_.readLine(resp))
        return false;

    return resp == "OK";
}

bool Servo::getPosition(float& angle_deg) {
    std::ostringstream cmd;
    cmd << "GET_POS " << id_;

    if (!serial_.writeLine(cmd.str()))
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
bool Servo::getEffort(float& effort) {
    std::ostringstream cmd;
    cmd << "GET_TORQUE " << id_;

    if (!serial_.writeLine(cmd.str()))
        return false;

    std::string resp;
    if (!serial_.readLine(resp))
        return false;

    try {
        effort = std::stof(resp);
    } catch (...) {
        return false;
    }
    return true;
}
bool Servo::getSpeed(float& speed) {
    std::ostringstream cmd;
    cmd << "GET_SPEED " << id_;

    if (!serial_.writeLine(cmd.str()))
        return false;

    std::string resp;
    if (!serial_.readLine(resp))
        return false;

    try {
        speed = std::stof(resp);
    } catch (...) {
        return false;
    }
    return true;        
}
bool Servo::grip(){
    std::ostringstream cmd;
    cmd << "GRIP " << id_ <<" 215 ";
    if (!serial_.writeLine(cmd.str()))
        return false;
    return true;
} 

bool Servo::readGrip(float& angle_deg){
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