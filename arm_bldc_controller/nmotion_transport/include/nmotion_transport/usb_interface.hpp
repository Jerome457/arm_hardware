/**
 * @file can_interface.hpp
 * @author Nawe Robotics
 * @brief File contains CAN Interface Class definations
 */

#pragma once
#include "interface.hpp"
#include <atomic>
#include <future>
#include <map>
#include <mutex>

class USBInterface : public Interface {
  /**
   * @brief Class defination for USB interface implementations
   *
   */
  class USBInterfaceImpl;

  /**
   * @brief  Pointer to class object which stores USB interface implementation
   *
   */
  USBInterfaceImpl *impl = NULL;

  /**
   * @brief Flag to check if the device is in flashing mode
   *
   */
  std::atomic<bool>did_request_msg{false};

  /**
   * @brief variable to store firmware version major
   */
  uint8_t fw_version_major_;
  
  /**
   * @brief variable to store firmware version minor
   */
  uint8_t fw_version_minor_;

  /**
   * @brief variable to store firmware version revision
   */
  uint8_t fw_version_revision_;

  /**
   * @brief variable to store firmware version commit
   */
  std::string fw_version_commit_;

  

  /**
   * @brief Parse can_frame from CAN Interface
   *
   */
  uint8_t recvData(struct can_frame_data *fra);

  /**
   * @brief Invokes lower level code to initlialize USB Interface
   *
   * @param interface_name Name of USB Interface eg. /dev/ttyACMO, COM5...etc
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
   * @brief Construct a new USBInterface object
   *
   */
  USBInterface();
  /**
   * @brief Destroy the USBInterface object
   *
   */
  ~USBInterface();
  /**
   * @brief Send a CAN msg frame on the interface
   *
   * @param fra CAN Msg frame
   * @return uint8_t Success(0) or Failure(1)
   */
  uint8_t sendData(struct can_frame_data *fra);

  /**
   * @brief Set CAN Baud Rate for NLink
   * 
   * @param baud_rate
   * @return uint8_t Success(0) or Failure(1)
   */
  uint8_t setCANBaudRate(CANBaudRate baud_rate);

  /**
   * @brief Set NLink to DFU Mode
   * 
   * @return uint8_t Success(0) or Failure(1)
   */
  uint8_t enterDFUMode();

  /**
   * @brief Get the firmware version of the device
   * 
   * @return uint8_t Success(0) or Failure(1)
   */
  uint8_t getVersions(uint8_t * fw_version_major, uint8_t* fw_version_minor, uint8_t* fw_version_revision, std::string * fw_version_commit);
};