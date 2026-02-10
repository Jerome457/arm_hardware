/**
 * @file can_device_type.hpp
 * @author Nawe Robotics
 * @brief CANDeviceType abstract class that is used to enforce the requirements
 * of CAN Device Type used in CAN Driver class
 */

#pragma once
#include <functional>
#include <stdint.h>

/**
 * @brief Custom CAN Frame structure to be used across interfaces
 *
 */
struct can_frame_data {
  uint32_t id;
  uint8_t dlc;
#if defined(_WIN32)
  __declspec(align(8)) uint8_t data[8];
#else
  uint8_t data[8] __attribute__((aligned(8)));
#endif
};

/**
 * @brief CAN Device Type Class
 *
 */
class CANDeviceType {
private:
  /**
   * @brief Copy constructor: Disable copy for CANDeviceType objects
   *
   */
  CANDeviceType(CANDeviceType const &);

  /**
   * @brief = operator overloading: Disable this for CANDeviceType objects
   *
   * @return CANDeviceType&
   */
  CANDeviceType &operator=(CANDeviceType const &);

protected:
  /**
   * @brief Device Id
   *
   */
  uint32_t id = -1;

public:
  /**
   * @brief Default Constructor for a new CANDeviceType object
   *
   */
  CANDeviceType();

  /**
   * @brief Callback function which is invoked when a CAN Msg Frame comes to
   * interface for this device (filtered using device id)
   *
   * @param fra Reference to the received CAN Msg frame in custom CAN Frame
   * format.
   */
  virtual void canMsgCallback(struct can_frame_data *fra) = 0;

  /**
   * @brief Get device id
   *
   * @return uint32_t device id
   */
  uint32_t getId();
};
