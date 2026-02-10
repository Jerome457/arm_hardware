/**
 * @file NCoder.hpp
 * @author Nawe Robotics
 * @brief File contains NCoder class definations
 */

#pragma once
#include "can_device_type.hpp"
#include "interface.hpp"

/**
 * @brief NCoder Class
 *
 */
class NCoder : public CANDeviceType
{

    /**
     * @brief Class defination for NCoder Implementation class
     *
     */
    class NcoderImpl;

    /**
     * @brief Pointer to class object which stores NCoder implementation
     *
     */
    NcoderImpl *impl;

    /**
     * @brief CAN Interface object reference. This object is used to send/receive
     * data from CAN Bus.
     *
     */
    Interface *iface;

    /**
     * @brief Callback function which is invoked when a new CAN msg for this
     * device comes into the interface
     *
     * @param fra Reference to the received CAN Msg frame
     */
    void canMsgCallback(struct can_frame_data *fra);

public:
    /**
     * @brief Construct a new NCoder object
     *
     * @param device_id Device Id for the NCoder device
     * @param interface CAN Interface / USB Interface object for the interface to which the
     * NCoder is connected
     */
    NCoder(uint32_t device_id, Interface *interface);

#ifndef _WIN32
    /**
     * @brief Construct a new NCoder object
     *
     * @param device_id Device Id for the NCoder device
     * @param interface CAN Interface / USB Interface object for the interface to which the
     * NCoder is connected
     */
    NCoder(Interface *interface);
#endif

    ~NCoder();

    /**
     * @brief Check if the device is still connected or not
     *
     * @return true
     * @return false
     */
    bool isConnected();

    /**
     * @brief Set the Output Log Stream object
     *
     * @param out Output log stream object
     */
    void setOutputLogStream(std::ostream *out);

    /**
     * @brief Get the hardware version of the NCoder device
     *
     * @param tag Pointer to the variable where the hardware tag is to be saved
     * @param major Pointer to the variable where the major version is to be saved
     * @param minor Pointer to the variable where the minor version is to be saved
     * @param variant Pointer to the variable where the variant is to be saved
     * @return Return Status
     */
    ret_status_t getHardwareVersion(uint8_t *tag, uint8_t *major, uint8_t *minor, uint8_t *variant);

    /**
     * @brief Get the firmware version of the NCoder device
     *
     * @param major Pointer to the variable where the major version is to be saved
     * @param minor Pointer to the variable where the minor version is to be saved
     * @param revision Pointer to the variable where the revision number is to be saved
     * @return Return Status
     */
    ret_status_t getFirmwareVersion(uint8_t *major, uint8_t *minor, uint16_t *revision);

    /**
     * @brief Get the firmware commit hash of the NCoder device
     *
     * @param commit Pointer to the string where the commit hash is to be saved
     * @return Return Status
     */
    ret_status_t getFirmwareCommit(std::string *commit);

    /**
     * @brief Set the node id for the NCoder device
     *
     * @param id CAN Node ID of the NCoder device
     * @return Return Status
     */
    ret_status_t setNodeId(uint32_t id);

    /**
     * @brief Get current CAN Node ID of the NCoder device
     *
     * @param id Pointer to the variable to which the current CAN Node ID of the
     * NCoder device is to be saved.
     * @return Return Status
     */
    ret_status_t getNodeId(uint32_t *node_id);

    /**
     * @brief Configure the encoder parameters
     *
     * @param encoder_type Set the encoder type
     * @param resolution_param Set the encoder resolution [in bits]
     * @return Return Status
     */
    ret_status_t configureEncoder(uint8_t encoder_type, int32_t resolution_param);

    /**
     * @brief Retrieve the current encoder configuration parameters
     *
     * @param encoder_type Pointer to the variable where the encoder type value is to be saved
     * @param resolution_param Pointer to the variable where the encoder resolution [in bits] is to be saved
     * @return Return Status
     */
    ret_status_t getEncoderConfiguration(uint8_t *encoder_type, int32_t *resolution_param);

    /**
     * @brief Set the current position of the encoder to zero
     *
     * @return Return Status
     */
    ret_status_t setCurrentPositionToZero();

    /**
     * @brief Get the zero position of the encoder
     *
     * @param zero_position Pointer to the variable where the zero position value is to be saved
     * @return Return Status
     */
    ret_status_t getZeroPosition(float *zero_position);

    /**
     * @brief Set the encoder direction
     *
     * @param direction Encoder direction to be set
     * @return Return Status
     */
    ret_status_t setEncoderDirection(uint8_t direction);

    /**
     * @brief Get the encoder direction
     *
     * @param direction Pointer to the variable where the encoder direction is to be saved
     * @return Return Status
     */
    ret_status_t getEncoderDirection(uint8_t *direction);

    /**
     * @brief Set the filter window length for the encoder
     *
     * @param filter_window_length Filter window length value to be set
     * @return Return Status
     */
    ret_status_t setFilterWindowLength(uint32_t filter_window_length);

    /**
     * @brief Get the filter window length for the encoder
     *
     * @param filter_window_length Pointer to the variable where the filter window length value is to be saved
     * @return Return Status
     */
    ret_status_t getFilterWindowLength(uint32_t *filter_window_length);

    /**
     * @brief Set a value in the MAxxx SPI register
     *
     * @param register_address Address of the register to be set
     * @param data Data to be written to the register
     * @return Return Status
     */
    ret_status_t setMAxxxSPIRegister(uint8_t register_address, uint8_t data);

    /**
     * @brief Get a value from the MAxxx SPI register
     *
     * @param register_address Address of the register to be read
     * @param data Pointer to the variable where the register data is to be saved
     * @param status Pointer to the variable where the status of the operation is to be saved
     * @return Return Status
     */
    ret_status_t getMAxxxSPIRegister(uint8_t register_address, uint8_t *data, bool *status);

    /**
     * @brief Set the scaling factor for the encoder
     *
     * @param scaling_factor Scaling factor to be set
     * @return Return Status
     */
    ret_status_t setScalingFactor(float scaling_factor);

    /**
     * @brief Get the scaling factor for the encoder
     *
     * @param scaling_factor Pointer to the variable where the scaling factor is to be saved
     * @return Return Status
     */
    ret_status_t getScalingFactor(float *scaling_factor);

    /**
     * @brief Get the current encoder count (angular Increment)
     *
     * @param encoder_count Pointer to the variable where the encoder count value is to be saved
     * @return Return Status
     */
    ret_status_t getEncoderCount(int32_t *encoder_count);

    /**
     * @brief Get the raw absolute position data directly from encoder hardware
     *
     * @param abs_position Pointer to the variable where the raw data value is to be saved
     * @return Return Status
     */
    ret_status_t getEncoderRawData(uint32_t *abs_position);

    /**
     * @brief Get the current absolute angle measurement from the encoder (in degrees)
     *
     * @param abs_angle Pointer to the variable where the absolute angle value is to be saved
     * @return Return Status
     */
    ret_status_t getAbsoluteAngle(float *abs_angle);

    /**
     * @brief Get the multiturn angle of the encoder
     *
     * @param multiturn_angle Pointer to the variable where the multiturn angle is to be saved
     * @return Return Status
     */
    ret_status_t getMultiturnAngle(float *multiturn_angle);

    /**
     * @brief Get the raw velocity value
     *
     * @param velocity_raw Pointer to the variable where the raw velocity value is to be saved
     * @return Return Status
     */
    ret_status_t getRawVelocity(float *velocity_raw);

    /**
     * @brief Get the scaled velocity value
     *
     * @param velocity_scaled Pointer to the variable where the scaled velocity value is to be saved
     * @return Return Status
     */
    ret_status_t getScaledVelocity(float *velocity_scaled);

    /**
     * @brief Get the filtered velocity value
     *
     * @param velocity_filtered Pointer to the variable where the filtered velocity value is to be saved
     * @return Return Status
     */
    ret_status_t getFilteredVelocity(float *velocity_filtered);

    /**
     * @brief Get the scaled and filtered velocity value
     *
     * @param velocity_scaled_filtered Pointer to the variable where the scaled and filtered velocity value is to be saved
     * @return Return Status
     */
    ret_status_t getScaledFilteredVelocity(float *velocity_scaled_filtered);

    /**
     * @brief Get the raw acceleration value
     *
     * @param acceleration_raw Pointer to the variable where the raw acceleration value is to be saved
     * @return Return Status
     */
    ret_status_t getRawAcceleration(float *acceleration_raw);

    /**
     * @brief Get the scaled acceleration value
     *
     * @param acceleration_scaled Pointer to the variable where the scaled acceleration value is to be saved
     * @return Return Status
     */
    ret_status_t getScaledAcceleration(float *acceleration_scaled);

    /**
     * @brief Get the filtered acceleration value
     *
     * @param acceleration_filtered Pointer to the variable where the filtered acceleration value is to be saved
     * @return Return Status
     */
    ret_status_t getFilteredAcceleration(float *acceleration_filtered);

    /**
     * @brief Get the scaled and filtered acceleration value
     *
     * @param acceleration_scaled_filtered Pointer to the variable where the scaled and filtered acceleration value is to be saved
     * @return Return Status
     */
    ret_status_t getScaledFilteredAcceleration(float *acceleration_scaled_filtered);

    /**
     * @brief Set the linear count for the encoder
     *
     * @param count Linear count value to be set
     * @return Return Status
     */
    ret_status_t setLinearCount(uint32_t count);

    /**
     * @brief Get the linear count for the encoder
     * @param error_coder Pointer to the variable where the error code is to be saved
     */
    ret_status_t getErrorCode(uint32_t *error_code);

    /**
     * @brief Clear the encoder error state
     */
    ret_status_t clearEncoderErrors();
    /**
     * @brief Reboot the NCoder device
     *
     * @return Return Status
     */
    ret_status_t rebootNCoder();

    /**
     * @brief Save the current configurations to non-volatile memory
     *
     * @return Return Status
     */
    ret_status_t saveConfigurations();

    /**
     * @brief Erase the saved configurations from non-volatile memory
     *
     * @return Return Status
     */
    ret_status_t eraseConfigurations();

    /**
     * @brief Enter the Device Firmware Update (DFU) mode
     *
     * @return Return Status
     */
    ret_status_t enterDFUMode();
};