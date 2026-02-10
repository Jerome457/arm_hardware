/**
 * @file interface.hpp
 * @author Nawe Robotics
 * @brief Interface abstract class that is used to enforce the requirements
 * of Interfaces used in device classes.
 */

#pragma once
#include "can_device_type.hpp"
#include <functional>
#include <future>
#include <iostream>
#include <map>
#include <stdint.h>
#include <vector>

/**
 * @brief Enum contains all the return status
 *
 */
typedef enum
{
  RETURN_OK,                   //< Status OK
  RETURN_ERROR,                //< Unclassified Error
  RETURN_TIMEOUT,              //< CAN Communication Timeout
  RETURN_INVALID_ID,           //< Device ID is invalid
  RETURN_DEVICE_NOT_CONNECTED, //< Device is not connected
  RETURN_INTERFACE_NOT_UP,     //< CAN Interface is not running
  RETURN_WRONG_ARGUMENT,       //< Argument is not valid
} ret_status_t;

/**
 * @brief Enum contains all the possible baud rates
 *
 */
typedef enum
{
  BAUDRATE_10K,
  BAUDRATE_20K,
  BAUDRATE_50K,
  BAUDRATE_100K,
  BAUDRATE_125K,
  BAUDRATE_250K,
  BAUDRATE_500K,
  BAUDRATE_750K,
  BAUDRATE_1M
} CANBaudRate;

/**
 * @brief Interface Type Class
 *
 */
class Interface
{
protected:
  /**
   * @brief Class defination for interface implementations
   *
   */
  class InterfaceImpl;
  /**
   * @brief Pointer to class object which stores interface implementation
   * details
   *
   */
  InterfaceImpl *impl = NULL;

  /**
   * @brief Mutex used to synchronise modifications to connected_devices map
   *
   */
  std::shared_ptr<std::mutex> mx_connected_devices;

  /**
   * @brief List of connected devices node ids with corresponding pair of hearbeat status and device type
   *
   */
  // std::map<uint32_t, std::pair<uint8_t, bool>> connected_devices = {};

  std::map<uint32_t, std::pair<uint8_t, std::chrono::steady_clock::time_point>> connected_devices = {};

  /**
   * @brief Invokes lower level code to initlialize Interface
   *
   * @param interface_name Name of Interface eg. can0, vcan0, ttyACM0...etc
   * @return uint8_t
   */
  virtual uint8_t init(std::string interface_name) = 0;

  /**
   * @brief Method specific to closing interface.
   *
   */
  virtual void closeIface(void) = 0;

  /**
   * @brief Configurable output stream which is used for logging
   *
   */
  std::ostream *output;

  /**
   * @brief Send a CAN msg frame on the interface
   *
   * @param fra CAN Msg frame
   * @return uint8_t Success(0) or Failure(1)
   */
  virtual uint8_t sendData(struct can_frame_data *fra) = 0;

  /**
   * @brief Interface specific implementation for getting can_frame_data from
   * interface.
   *
   * @param fra Pointer to can_frame_data where received data from interface
   * will be stored.
   * @return uint8_t Success(0) or Failure(1)
   */
  virtual uint8_t recvData(struct can_frame_data *fra) = 0;

public:
  bool is_device_flashing = false;
  /**
   * @brief Construct a new Interface object
   *
   */
  Interface();
  /**
   * @brief Destroy the Interface object
   *
   */
  ~Interface();

  /**
   * @brief Check if the interface is still up and running
   *
   * @return true
   * @return false
   */
  bool isRunning();

  /**
   * @brief Invokes lower level interface initialisation and starts various
   * threads
   *
   * @param interface_name Name of CAN Interface eg. can0, vcan0...etc
   * @param on_new_dev_cb Callback function which will be called when a new
   * device is connected
   * @param on_dev_disconnect_cb Callback function which will be called whem an existing device is disconnected.
   * @return uint8_t Success(0) or Failure(1)
   */
  uint8_t
  initInterface(std::string interface_name,
                std::function<void(uint32_t, uint8_t)> on_new_dev_cb = nullptr,
                std::function<void(uint32_t)> on_dev_disconnect_cb = nullptr);

  /**
   * @brief Closes interface and running threads
   *
   * @return uint8_t Success(0) or Failure(1)
   */
  uint8_t closeInterface(void);

  /**
   * @brief Set the Output Log Stream object
   *
   * @param out Output log stream object
   */
  void setOutputLogStream(std::ostream *out);

  /**
   * @brief Register a device callback to callback list
   *
   * @param can_dev CAN Device Object
   */
  void registerCANMsgCallback(CANDeviceType *can_dev);

  /**
   * @brief Unregister a device callback from callback list
   *
   * @param can_dev CAN Device Object
   */
  void unregisterCANMsgCallback(CANDeviceType *can_dev);

  /**
   * @brief Get list of connected devices
   *
   * @return std::map<uint32_t, uint8_t> map array of node id's and corresponding device types which are connected.
   */
  std::map<uint32_t, uint8_t> getConnectedDevices();

  /**
   * @brief Send Frame via the interface
   *
   * @param fra Frame to be send
   * @return uint8_t Success(0) or Failure(1)
   */
  uint8_t sendFrame(struct can_frame_data *fra);

  /**
   * @brief Set CAN Baud Rate at the interface
   *
   * @param baud_rate
   * @return uint8_t Success(0) or Failure(1)
   */
  virtual uint8_t setCANBaudRate(CANBaudRate baud_rate) = 0;

  /**
   * @brief Check if the given device is NLink or not
   *
   * @return true
   * @return false
   */
  bool isDevice();

  /**
   * @brief Set the Interface as connected to device
   * 
   * @return uint32_t Node ID of the connected device 
   */
  uint32_t setAsDevice();

  /**
   * @brief Check if the given device is connected or not.
   *
   * @param dev_id
   * @return true
   * @return false
   */
  bool isDeviceConnected(uint32_t dev_id);
};
