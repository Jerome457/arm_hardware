/**
 * @file imu.hpp
 * @author Nawe Robotics
 * @brief File contains IMU class definations
 */

#pragma once
#include "can_device_type.hpp"
#include "interface.hpp"

/**
 * @brief IMU Class
 *
 */
class IMU : public CANDeviceType {

    /**
     * @brief Class defination for IMU Implementation class
     *
     */
    class IMUImpl;

    /**
     * @brief Pointer to class object which stores IMU implementation
     *
     */
    IMUImpl *impl;

    /**
     * @brief CAN Interface object reference. This object is used to send/receive
     * data from CAN Bus.
     *
     */
    Interface *iface;

    enum{
        ENABLE_ROTATION_VECTOR =0x01,
        ENABLE_RAW_ACCELEROMETER,
        ENABLE_RAW_GYRO,
        ENABLE_RAW_MAGNETOMETER,
        ENABLE_LINEAR_ACCELEROMETER,
        ENABLE_ACCELEROMETER,
        ENABLE_GYRO,
        ENABLE_MAGNETOMETER,
        ENABLE_UNCALIBRATED_GYRO,
        ENABLE_GRAVITY,
    };

    /**
     * @brief Callback function which is invoked when a new CAN msg for this
     * device comes into the interface
     *
     * @param fra Reference to the received CAN Msg frame
     */
    void canMsgCallback(struct can_frame_data *fra);
public:
    /**
     * @brief Construct a new IMU object
     *
     * @param device_id Device Id for the IMU device
     * @param interface CAN Interface object for the interface to which the
     * IMU is connected
     */
    IMU(uint32_t device_id, Interface *interface);


    /**
     * @brief Construct a new IMU object
     *
     * @param device_id Device Id for the IMU device
     * @param interface CAN Interface object for the interface to which the
     * IMU is connected
     */
    IMU(Interface *interface);

    /**
     * @brief Set the Output Log Stream object
     *
     * @param out Output log stream object
     */
    void setOutputLogStream(std::ostream *out);

    /**
     * @brief Enable Rotation Vector Data and configure it
     * @param report_interval Report interval in milliseconds
     * @return Return Status
     */
    ret_status_t enableRotationVector(uint16_t report_interval);

    /**
     * @brief Disable Rotation Vector Data
     * @return Return Status
     */
    ret_status_t disableRotationVector();

    /**
     * @brief Enable Raw Accelerometer Data and configure it
     * @param report_interval Report interval in milliseconds
     * @return Return Status
     */
    ret_status_t enableRawAccelerometer(uint16_t report_interval);

    /**
     * @brief Disable Raw Accelerometer Data
     * @return Return Status
     */
    ret_status_t disableRawAccelerometer();

    /**
     * @brief Enable Raw Gyroscope Data and configure it
     * @param report_interval Report interval in milliseconds
     * @return Return Status
     */
    ret_status_t enableRawGyro(uint16_t report_interval);

    /**
     * @brief Disable Raw Gyroscope Data
     * @return Return Status
     */
    ret_status_t disableRawGyro();

    /**
     * @brief Enable Raw Magnetometer Data and configure it
     * @param report_interval Report interval in milliseconds
     * @return Return Status
     */
    ret_status_t enableRawMagnetometer(uint16_t report_interval);

    /**
     * @brief Disable Raw Magnetometer Data
     * @return Return Status
     */
    ret_status_t disableRawMagnetometer();

    /**
     * @brief Enable Linear Accelerometer Data and configure it
     * @param report_interval Report interval in milliseconds
     * @return Return Status
     */
    ret_status_t enableLinearAccelerometer(uint16_t report_interval);

    /**
     * @brief Disable Linear Accelerometer Data
     * @return Return Status
     */
    ret_status_t disableLinearAccelerometer();

    /**
     * @brief Enable Accelerometer Data and configure it
     * @param report_interval Report interval in milliseconds
     * @return Return Status
     */
    ret_status_t enableAccelerometer(uint16_t report_interval);

    /**
     * @brief Disable Accelerometer Data
     * @return Return Status
     */
    ret_status_t disableAccelerometer();

    /**
     * @brief Enable Gyroscope Data and configure it
     * @param report_interval Report interval in milliseconds
     * @return Return Status
     */
    ret_status_t enableGyro(uint16_t report_interval);

    /**
     * @brief Disable Gyroscope Data
     * @return Return Status
     */
    ret_status_t disableGyro();

    /**
     * @brief Enable Magnetometer Data and configure it
     * @param report_interval Report interval in milliseconds
     * @return Return Status
     */
    ret_status_t enableMagnetometer(uint16_t report_interval);

    /**
     * @brief Disable Magnetometer Data
     * @return Return Status
     */
    ret_status_t disableMagnetometer();

    /**
     * @brief Enable Uncalibrated Gyroscope Data and configure it
     * @param report_interval Report interval in milliseconds
     * @return Return Status
     */
    ret_status_t enableUncalibratedGyro(uint16_t report_interval);

    /**
     * @brief Disable Uncalibrated Gyroscope Data
     * @return Return Status
     */
    ret_status_t disableUncalibratedGyro();

    /**
     * @brief Enable Gravity Data and configure it
     * @param report_interval Report interval in milliseconds
     * @return Return Status
     */
    ret_status_t enableGravity(uint16_t report_interval);

    /**
     * @brief Disable Gravity Data
     * @return Return Status
     */
    ret_status_t disableGravity();
    
    /**
     * @brief Retrieves the rotation vector data from the IMU.
     * @param quat_i Pointer to the variable where the i component of the quaternion is to be stored.
     * @param quat_j Pointer to the variable where the j component of the quaternion is to be stored.
     * @param quat_k Pointer to the variable where the k component of the quaternion is to be stored.
     * @param quat_real Pointer to the variable where the real component of the quaternion is to be stored.
     * @return Status of the operation.
    */
    ret_status_t getRotationVectorData(float* quat_i, float* quat_j, float* quat_k, float* quat_real);

    /**
     * @brief Retrieves the rotation vector accuracy data from the IMU.
     * @param quat_rad_accuracy Pointer to the variable where the estimated accuracy of the quaternion in radians is to be stored.
     * @param quat_accuracy Pointer to the variable where the estimated accuracy of the quaternion is to be stored.
     * @return Status of the operation.
    */
    ret_status_t getRotationVectorAccuracy(float* quat_rad_accuracy, uint8_t* quat_accuracy );

    /**
     * @brief Retrieves the raw accelerometer data from the IMU.
     * @param x Pointer to the variable where the raw accelerometer value in x axis is to be stored.
     * @param y Pointer to the variable where the raw accelerometer value in y axis is to be stored.
     * @param z Pointer to the variable where the raw accelerometer value in z axis is to be stored.
     * @return Status of the operation.
    */
    ret_status_t getRawAccelerometerData(int16_t* x, int16_t* y, int16_t* z);

    /**
     * @brief Retrieves the raw gyroscope data from the IMU.
     * @param x Pointer to the variable where the raw gyro value in x axis is to be stored.
     * @param y Pointer to the variable where the raw gyro value in y axis is to be stored.
     * @param z Pointer to the variable where the raw gyro value in z axis is to be stored.
     * @return Status of the operation.
    */
    ret_status_t getRawGyroData(int16_t* x, int16_t* y, int16_t* z);

    /**
     * @brief Retrieves the raw magnetometer data from the IMU.
     * @param x Pointer to the variable where the raw magnetometer value in x axis is to be stored.
     * @param y Pointer to the variable where the raw magnetometer value in y axis is to be stored.
     * @param z Pointer to the variable where the raw magnetometer value in z axis is to be stored.
     * @return Status of the operation.
    */
    ret_status_t getRawMagnetometerData(int16_t* x, int16_t* y, int16_t* z);

    /**
     * @brief Retrieves the linear accelerometer data from the IMU.
     * @param x Pointer to the variable where the linear acceleration in x axis (m/s²) is to be stored.
     * @param y Pointer to the variable where the linear acceleration in y axis (m/s²) is to be stored.
     * @param z Pointer to the variable where the linear acceleration in z axis (m/s²) is to be stored.
     * @param accuracy Pointer to the variable where the accuracy of the linear acceleration data is to be stored.
     * @return Status of the operation.
    */
    ret_status_t getLinearAccelerometerData(float* x, float* y, float* z, uint8_t* accuracy);

    /**
     * @brief Retrieves the accelerometer data from the IMU.
     * @param x Pointer to the variable where the acceleration in x axis (m/s²) is to be stored.
     * @param y Pointer to the variable where the acceleration in y axis (m/s²) is to be stored.
     * @param z Pointer to the variable where the acceleration in z axis (m/s²) is to be stored.
     * @param accuracy Pointer to the variable where the accuracy of the acceleration data is to be stored.
     * @return Status of the operation.
    */
    ret_status_t getAccelerometerData(float* x, float* y, float* z, uint8_t* accuracy);
    
    /**
     * @brief Retrieves the gyroscope data from the IMU.
     * @param x Pointer to the variable where the gyro value in x axis (radians per second) is to be stored.
     * @param y Pointer to the variable where the gyro value in y axis (radians per second) is to be stored.
     * @param z Pointer to the variable where the gyro value in z axis (radians per second) is to be stored.
     * @param accuracy Pointer to the variable where the accuracy of the gyro data is to be stored.
     * @return Status of the operation.
    */
    ret_status_t getGyroData(float* x, float* y, float* z, uint8_t* accuracy);

    /**
     * @brief Retrieves the magnetometer data from the IMU.
     * @param x Pointer to the variable where the magnetometer value in x axis (uTesla) is to be stored.
     * @param y Pointer to the variable where the magnetometer value in y axis (uTesla) is to be stored.
     * @param z Pointer to the variable where the magnetometer value in z axis (uTesla) is to be stored.
     * @param accuracy Pointer to the variable where the accuracy of the magnetometer data is to be stored.
     * @return Status of the operation.
    */
    ret_status_t getMagnetometerData(float* x, float* y, float* z, uint8_t* accuracy);

    /**
     * @brief Retrieves the uncalibrated gyroscope data from the IMU.
     * @param x Pointer to the variable where the uncalibrated gyro value in x axis (radians per second) is to be stored.
     * @param y Pointer to the variable where the uncalibrated gyro value in y axis (radians per second) is to be stored.
     * @param z Pointer to the variable where the uncalibrated gyro value in z axis (radians per second) is to be stored.
     * @return Status of the operation.
    */
    ret_status_t getUncalibratedGyroData(float* x, float* y, float* z);

    /**
     * @brief Retrieves the uncalibrated gyroscope bias data from the IMU.
     * @param x Pointer to the variable where the uncalibrated gyro bias in x axis (radians per second) is to be stored.
     * @param y Pointer to the variable where the uncalibrated gyro bias in y axis (radians per second) is to be stored.
     * @param z Pointer to the variable where the uncalibrated gyro bias in z axis (radians per second) is to be stored.
     * @return Status of the operation.
    */
    ret_status_t getUncalibratedGyroBiasData(float* x, float* y, float* z);

    /**
     * @brief Retrieves the Euler angles from the IMU.
     * @param yaw Pointer to the variable where the yaw angle (degrees) is to be stored.
     * @param pitch Pointer to the variable where the pitch angle (degrees) is to be stored.
     * @param roll Pointer to the variable where the roll angle (degrees) is to be stored.
     * @return Status of the operation.
    */
    ret_status_t getEulerAngles(float* yaw, float* pitch, float* roll);

    /**
     * @brief Retrieves the gravity data from the IMU.
     * @param x Pointer to the variable where the gravity in x axis (g) is to be stored.
     * @param y Pointer to the variable where the gravity in y axis (g) is to be stored.
     * @param z Pointer to the variable where the gravity in z axis (g) is to be stored.
     * @return Status of the operation.
    */
    ret_status_t getGravityData(float* x, float* y, float* z);

    /**
     * @brief Set the node id for the NIMU
     *
     * @param id CAN Node ID of the NIMU
     * @return Return Status
    */
    ret_status_t setNodeId(uint32_t id);

    /**
     * @brief Get current CAN Node ID of the NIMU
     *
     * @param id Pointer to the variable to which the current CAN Node ID of the
     * NIMU is to be saved.
     * @return Return Status
    */
    ret_status_t getNodeId(uint32_t *node_id);

     /**
     * @brief Get the hardware version of the NIMU device
     *
     * @param tag Pointer to the variable where the hardware tag is to be saved
     * @param major Pointer to the variable where the major version is to be saved
     * @param minor Pointer to the variable where the minor version is to be saved
     * @param variant Pointer to the variable where the variant is to be saved
     * @return Return Status
     */
    ret_status_t getHardwareVersion(uint8_t *tag, uint8_t *major, uint8_t *minor, uint8_t *variant);

    /**
     * @brief Get the firmware version of the NIMU device
     *
     * @param major Pointer to the variable where the major version is to be saved
     * @param minor Pointer to the variable where the minor version is to be saved
     * @param revision Pointer to the variable where the revision number is to be saved
     * @return Return Status
     */
    ret_status_t getFirmwareVersion(uint8_t *major, uint8_t *minor, uint16_t *revision);

    /**
     * @brief Get the firmware commit hash of the NIMU device
     *
     * @param commit Pointer to the string where the commit hash is to be saved
     * @return Return Status
     */
    ret_status_t getFirmwareCommit(std::string *commit);

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
     * @brief Reboot the NIMU device
     *
     * @return Return Status
     */
    ret_status_t rebootNIMU();
    
    /**
     * @brief Start calibration and configure it
     * @param accel Enable accelerometer calibration
     * @param gyro Enable gyroscope calibration
     * @param mag Enable magnetometer calibration
     * @param planarAccel Enable planar accelerometer calibration
     * @return Return Status
     */
    ret_status_t startCalibration(bool accel, bool gyro, bool mag, bool planarAccel);

    /**
     * @brief Stop the ongoing calibration process
     * @return Return Status
     */
    ret_status_t stopCalibration();

    /**
     * @brief Save calibration data to non-volatile memory
     * @return Return Status
     */
    ret_status_t saveCalibration();

    /**
     * @brief Get the current calibration status
     * @param isCalibrated Pointer to the variable where calibration completion status is to be stored
     * @return Return Status
     */
    ret_status_t getCalibrationStatus(bool* isCalibrated);

    /**
     * @brief Clear Dynamic Calibration Data (DCD)
     * @return Return Status
     */
    ret_status_t clearDCD();


    /**
     * @brief Apply tare to all axes of the IMU
     * @return Return Status
     */
    ret_status_t tareAll();
    
    /**
     * @brief Apply tare to the Z-axis
     * @return Return Status
     */
    ret_status_t tareYaw();
    
    /**
     * @brief Clear all tare offsets
     * @return Return Status
     */
    ret_status_t clearTare();
    
    /**
     * @brief Set the sensor orientation using a quaternion
     * @param i The i component of the quaternion
     * @param j The j component of the quaternion
     * @param k The k component of the quaternion
     * @param real The real component of the quaternion
     * @return Return Status
     */
    ret_status_t setOrientation(float i, float j, float k, float real);

    /**
     * @brief Enter the Device Firmware Update (DFU) mode
     *
     * @return Return Status
     */
    ret_status_t enterDFUMode();
    
    /**
     * @brief Destroy the IMU object
     *
     */
    ~IMU();

    /**
     * @brief Check if the device is still connected or not
     *
     * @return true
     * @return false
     */
    bool isConnected();

};