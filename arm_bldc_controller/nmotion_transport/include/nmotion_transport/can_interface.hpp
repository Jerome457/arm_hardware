/**
 * @file can_interface.hpp
 * @author Nawe Robotics
 * @brief File contains CAN Interface Class definations
 */

#pragma once
#include "can_device_type.hpp"
#include "interface.hpp"
#include <functional>
#include <future>
#include <iostream>
#include <map>
#include <vector>

/**
 * @brief CAN Interface class
 *
 */
class CANInterface : public Interface {
private:
  /**
   * @brief Class defination for CAN interface implementations
   *
   */
  class CANInterfaceImpl;

  /**
   * @brief  Pointer to class object which stores CAN interface implementation
   *
   */
  CANInterfaceImpl *impl = NULL;

  /**
   * @brief Parse can_frame from CAN Interface
   *
   */
  uint8_t recvData(struct can_frame_data *fra);

  /**
   * @brief Invokes lower level code to initlialize CAN Interface
   *
   * @param interface_name Name of CAN Interface eg. can0, vcan0...etc
   * @return uint8_t
   */
  uint8_t init(std::string interface_name);

  /**
   * @brief Method specific to closing CAN interface.
   *
   */
  void closeIface(void);

public:
  /**
   * @brief Construct a new CANInterface object
   *
   */
  CANInterface();
  /**
   * @brief Destroy the CANInterface object
   *
   */
  ~CANInterface();

  /**
   * @brief Send a CAN msg frame on the interface
   *
   * @param fra CAN Msg frame
   * @return uint8_t Success(0) or Failure(1)
   */
  uint8_t sendData(struct can_frame_data *fra);

  /**
   * @brief Set the Baud Rate
   * 
   * @return uint8_t 
   */
  uint8_t setCANBaudRate(CANBaudRate baud_rate);
};
