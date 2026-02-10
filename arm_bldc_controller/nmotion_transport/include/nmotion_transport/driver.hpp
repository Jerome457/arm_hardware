/**
 * @file driver.hpp
 * @author Nawe Robotics
 * @brief File contains Driver class definations
 */

#pragma once
#include "can_device_type.hpp"
#include "interface.hpp"

/**
 * @brief Driver Class
 *
 */
class Driver : public CANDeviceType {

  /**
   * @brief Class defination for Driver Implementation class
   *
   */
  class DriverImpl;

  /**
   * @brief Pointer to class object which stores Driver implementation
   *
   */
  DriverImpl *impl;

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
   * @brief Construct a new Driver object
   *
   * @param node_id CAN Node ID of the driver
   * @param interface Initialised CAN Interface / USB Interface object for the interface to
   * which the driver is connected
   */
  Driver(uint32_t node_id, Interface *interface);

#ifndef _WIN32
    /**
     * @brief Construct a new Driver object
     *
     * @param device_id Device Id for the NCoder device
     * @param interface CAN Interface / USB Interface object for the interface to which the
     * Driver is connected
     */
    Driver(Interface *interface);
#endif
  /**
   * @brief Set an output log stream object to which the low-level logs can be
   * written. To be used for debugging purposes.
   *
   * @param out Output log stream object
   */
  void setOutputLogStream(std::ostream *out);

  /*Controller CAN Functions*/
  /**
   * @brief Get Driver's Hardware version
   *
   * @param tag Pointer to the variable where the hardware tag value is to be
   * saved.
   * @param major Pointer to the variable where the major version value is to be
   * saved.
   * @param minor Pointer to the variable where the minor version value is to be
   * saved.
   * @param variant Pointer to the variable where the variant value is to be
   * saved.
   * @return Return Status
   */
  ret_status_t getHardwareVersion(uint8_t *tag, uint8_t *major, uint8_t *minor,
                                  uint8_t *variant);

  /**
   * @brief Get Driver's Firmware version
   *
   * @param major Pointer to the variable where the major version value is to be
   * saved.
   * @param minor Pointer to the variable where the minor version value is to be
   * saved.
   * @param revision Pointer to the variable where the revsion value is to be
   * saved.
   * @return Return Status
   */
  ret_status_t getFirmwareVersion(uint8_t *major, uint8_t *minor,
                                  uint16_t *revision);

  /**
   * @brief Get Driver's Firmware commit
   *
   * @param commit Pointer to the variable where the commit value is to be
   * saved.
   * @return Return Status
   */
  ret_status_t getFirmwareCommit(std::string *commit);

  /**
   * @brief Function to issue an emergency stop to the device which halts the
   * driver and sets the error flag; Since the error flag is set the driver
   * doesn't respond to any other commands, until the driver is rebooted.
   *
   * @return Return Status
   */
  ret_status_t emergencyStop();

  /**
   *  @brief Set the driver to idle state
   *  @return Return Status
   */
  ret_status_t setDeviceToIdle();

  /**
   *  @brief Set the driver to active state
   *  @return Return Status
   */
  ret_status_t setDeviceToActive();

  /**
   *  @brief Set the driver's undervoltage and overvoltage trip levels. The
   * device will throw an error when the bus voltage will drop below
   * under-voltage level or goes higher that the over-voltage level. Used to
   * keep the driver safe from high transient currents and voltage
   * fluctuations.
   *
   *  @param undervoltage_level under-voltage trip level
   *  @param overvoltage_level over-voltage trip level
   *  @return Return Status
   */
  ret_status_t setDCBusTripLevels(float undervoltage_level,
                                  float overvoltage_level);

  /**
   *  @brief Get the driver's current under-voltage and over-voltage levels.
   *
   *  @param undervoltage_level Pointer to the variable where the under-voltage
   * trip level is to be saved.
   *  @param overvoltage_level Pointer to the variable where the over-voltage
   * trip level is to be saved.
   *  @return Return Status
   */
  ret_status_t getDCBusTripLevels(float *undervoltage_level,
                                  float *overvoltage_level);

  /**
   * @brief Enable brake resistor and configure it
   *
   * @param brake_resistance_value brake resistance value
   * @return Return Status
   */
  ret_status_t enableBrakeResistor(float brake_resistance_value);

  /**
   * @brief Disable brake resistor and configure it
   *
   * @return Return Status
   */
  ret_status_t disableBrakeResistor();

  /**
   * @brief Get current state of brake resistor
   *
   * @param enable  Pointer to variable to which the enable status of the brake
   * resistor is to be saved (true means that the brake resistor is enabled).
   * @param brake_resistance_value Pointer to the variable to which the value of
   * brake resistance is to be saved.
   * @param brake_resistor_armed  Pointer to the variable to which the value of
   * the arm status of the brake resistor is to be saved (true means that the
   * brake resistor is armed).
   * @param brake_resistor_saturated  Pointer to the variable to which the value
   * of saturation status of the brake resistor is to be saved (true means that
   * the brake resistor is saturated).
   * @return Return Status
   */
  ret_status_t getBrakeResistorState(bool *enable, float *brake_resistance_value,
                                     bool *brake_resistor_armed,
                                     bool *brake_resistor_saturated);

  /**
   * @brief Set the maximum regenerative current the power supply/battery can take.
   * The device will throw DC_BUS_OVERREGEN Error if the current sink is more than this value
   *
   * @param regen_current_trip_level maximum regeneration current that battery/supply
   * can take in
   * @return Return Status
   */
  ret_status_t setRegenCurrentTripLevel(float regen_current_trip_level);

  /**
   * @brief Get the maximum regenerative current the power supply/battery can take.
   *
   * @param regen_current_trip_level Pointer to the variable to which the value of the maximum regeneration current
   * that battery/supply is to be saved
   * @return Return Status
   */
  ret_status_t getRegenCurrentTripLevel(float *regen_current_trip_level);

  /**
   * @brief  Set parameters of the motor which is connected to the driver
   *
   * @param pole_pairs pole-pair value of the motor
   * @param kV_rating kV rating value of the motor
   * @param current_limit current limit value for the motor
   * @return Return Status
   */
  ret_status_t setMotorParameters(uint8_t pole_pairs, uint16_t kV_rating,
                                  float current_limit);

  /**
   * @brief Get current motor parameters
   *
   * @param pole_pairs Pointer to the variable to which the pole-pair value of
   * the motor is to be saved.
   * @param kV_rating Pointer to the variable to which the kV rating value of
   * the motor is to be saved.
   * @param current_limit Pointer to the variable to which the current limit
   * value for the motor is to be saved.
   * @return Return Status
   */
  ret_status_t getMotorParameters(uint8_t *pole_pairs, uint16_t *kV_rating,
                                  float *current_limit);

  /**
   * @brief Set Motor Calibration Parameters. These values are used to calculate
   * the phase resistance & inductance values.
   *
   * @param motor_calib_voltage calibration voltage used to calibrate motor
   * @param motor_calib_current calibration current used to calibrate motor
   * @return Return Status
   */
  ret_status_t setMotorCalibrationParameters(float motor_calib_voltage,
                                             float motor_calib_current);

  /**
   * @brief Get current Motor Calibration Parameters
   *
   * @param motor_calib_voltage Pointer to the variable to which the current
   * calibration voltage to calibrate motor is to be saved.
   * @param motor_calib_current Pointer to the variable to which the current
   * calibration current to calibrate motor is to be saved.
   * @return Return Status
   */
  ret_status_t getMotorCalibrationParameters(float *motor_calib_voltage,
                                             float *motor_calib_current);

  // Add a method to get motor phase resistance and inductance.

  /**
   * @brief Configure motor side encoder. Set it to onboard absolute, external
   * incremental or external absolute. Output side and motor side encoders can't
   * be set to same.
   *
   * @param encoder_type type of encoder used
   * @param resolution_param CPR value of the encoder in case of an incremental encoder or 
   * resolution value in 2^bits in case of absolute encoder 
   * (this parameter is ignored in the case of using onboard absolute encoder)
   * resolution_param is set to 4096 by default.
   * @param use_index flag to set whether to use the index pin for incremental
   * encoder or not. Valid only when an incremental encoder is used. use_index
   * is set to true by default.
   * @return Return Status
   */

  ret_status_t configureMotorEncoder(uint8_t encoder_type, int32_t resolution_param = 4096,
                                     bool use_index = true);

  /**
   * @brief Get motor encoder configuration
   *
   * @param encoder_type Pointer to the variable to which the current encoder
   * type value is to be saved. If the value is 0, it means the output encoder
   * is not enabled.
   * @param resolution_param Pointer to the variable to which the value of
   * encoder CPR (in case of incremental encoder) or value of encoder resolution
   * bits (in case of absolute encoder) is to be saved.
   * @param use_index Pointer to the variable to which the status of using index
   * pin for incremental encoder is to be saved.
   * @return Return Status
   */
  ret_status_t getMotorEncoderConfiguration(uint8_t *encoder_type,
                                            int32_t *resolution_param,
                                            bool *use_index);

  /**
   * @brief Set gear ratio (Ouput / Input) of the Gearbox.
   * By default the value is 1 in the driver.
   *
   * @param gear_ratio gear ratio of the gearbox
   * @return Return Status
   */
  ret_status_t setGearBoxParameters(float gear_ratio);

  /**
   * @brief Get Gear ratio of the Gearbox
   *
   * @param gear_ratio Pointer to the variable to which the gear ratio value is
   * to be saved.
   * @return Return Status
   */
  ret_status_t getGearBoxParameters(float *gear_ratio);

  /**
   * @brief Configure Output side encoder. Set it to onboard absolute, external
   * incremental or external absolute. Output side and motor side encoders can't
   * be set to same.
   *
   * @param encoder_type type of encoder used
   * @param resolution_param CPR value of the encoder in case of an incremental encoder or 
   * resolution value in 2^bits in case of absolute encoder 
   * (this parameter is ignored in the case of using onboard absolute encoder)
   * resolution_param is set to 4096 by default.
   * @param use_index flag to set whether to use the index pin for incremental
   * encoder or not. Valid only when an incremental encoder is used. use_index
   * is set to true by default.
   * @return Return Status
   */
  ret_status_t configureOutputEncoder(uint8_t encoder_type, int32_t resolution_param = 4096,
                                      bool use_index = true);

  /**
   * @brief Disable Output Encoder.
   *
   * @return Return Status
   */
  ret_status_t disableOutputEncoder();

  /**
   * @brief Get output encoder configuration
   *
   * @param encoder_type Pointer to the variable to which the current encoder
   * type value is to be saved. If the value is 0, it means the output encoder
   * is not enabled.
   * @param resolution_param Pointer to the variable to which the value of
   * encoder CPR (in case of incremental encoder) or value of encoder resolution
   * bits (in case of absolute encoder) is to be saved.
   * @param use_index Pointer to the variable to which the status of using index
   * pin for incremental encoder is to be saved.
   * @return Return Status
   */
  ret_status_t getOutputEncoderConfiguration(uint8_t *encoder_type,
                                             int32_t *resolution_param,
                                             bool *use_index);

  /**
   * @brief Function to set Motor Thermistor Parameters
   *
   * @param r_ref Reference Resistance at 25 degrees celsius of the NTC
   * themistor
   * @param beta beta value of the thermistor
   * @return Return Status
   */
  ret_status_t setMotorThermistorParameters(float r_ref, uint16_t beta);

  /**
   * @brief Enable Motor based Thermal limit. A thermistor is used
   * to measure motor temperature and start limiting the
   * motor current once the temperature starts to approach the upper limit. The
   * current limiting starts when the difference between the current temperature
   * & the upper limit reaches 20 degrees celsius. The acutators will shut
   * themselves off in case the temperature reaches the upper limit.
   *
   * @param upper_limit Shutdown temperature at which the driver will stop.
   * @return Return Status
   */
  ret_status_t enableMotorThermalLimit(uint16_t upper_limit);

  /**
   * @brief Disable Motor based Thermal limit.
   *
   * @return Return Status
   */
  ret_status_t disableMotorThermalLimit();

  /**
   * @brief Get the Motor thermistor configuration
   *
   * @param enable Pointer to the variable where the enabled/disabled status of
   * the motor thermal limit is to be saved.
   * @param r_ref Pointer to the variable to which the Reference Resistance
   * value is to be saved.
   * @param beta Pointer to the variable to which the beta value of thermistor
   * is to be saved.
   * @param upper_limit  Pointer to the variable where the upper limit of the
   * thermal limit is to be saved.
   * @param lower_limit Pointer to the variable where the lower limit of the
   * thermal limit is to be saved. This is the limit from which the current
   * limiting of the driver will start.
   * @return Return Status
   */
  ret_status_t getMotorThermistorConfiguration(bool *enable, float *r_ref,
                                               uint16_t *beta,
                                               uint8_t *upper_limit,
                                               uint8_t *lower_limit);

  /**
   * @brief Set the torque limit for the driver
   * @param torque_limit torque limit for controller (value in Nm)
   * @return Return Status
   */
  ret_status_t setTorqueLimit(float torque_limit);

  /**
   * @brief Get the torque limit for the driver
   * @param torque_limit Pointer to the variable to which the torque limit value
   * is to be saved.
   * @return Return Status
   */
  ret_status_t getTorqueLimit(float *torque_limit);
  /**
   *  @brief Function to run incremental encoder index search
   *  @return Return Status
   */
  ret_status_t runEncoderIndexSearch();

  /**
   *  @brief Function to run matches the motor encoder with output encoder
   *  @return Return Status
   */
  ret_status_t runEncoderMatching();

  /**
   *  @brief Run calibration sequence on the acutator.
   *  @return Return Status
   */
  ret_status_t runCalibrationSequence();

  /**
   *  @brief Run encoder calibration on the acutator.
   *  @return Return Status
   */
  ret_status_t runEncoderCalibration();

  /**
   * @brief Set the node id for the driver
   *
   * @param id CAN Node ID of the driver
   * @return Return Status
   */
  ret_status_t setNodeId(uint32_t id);

  /**
   * @brief Get current CAN Node ID of the driver
   *
   * @param node_id Pointer to the variable to which the current CAN Node ID of the
   * driver is to be saved.
   * @return Return Status
   */
  ret_status_t getNodeId(uint32_t *node_id);

  /**
   * @brief Set startup configuration for the driver
   *
   * @param index_search enable/disable index searching on startup
   * @param encoder_matching enable/disable output encoder matching sequence on
   * startup
   * @param closed_loop enable/disable setting driver to closed loop on startup
   * @return Return Status
   */
  ret_status_t setStartupConfigs(bool index_search, bool encoder_matching,
                                 bool closed_loop);

  /**
   * @brief Get startup configuration of the driver
   *
   * @param index_search Pointer to the variable to which the enable/disable
   * status of index searching on startup is to be saved.
   * @param encoder_matching Pointer to the variable to which the enable/disable
   * status of output encoder matching sequence on startup is to be saved.
   * @param closed_loop  Pointer to the variable to which the enable/disable
   * status of setting the driver to closed loop on startup value is to be
   * saved.
   * @return Return Status
   */
  ret_status_t getStartupConfigs(bool *index_search, bool *encoder_matching,
                                 bool *closed_loop);

  /**
   * @brief Set current position of the driver as zero
   *
   * @return Return Status
   */
  ret_status_t setCurrentPostionToZero();

  /**
   * @brief Get zero offset postion in degrees
   *
   * @param zero_position Pointer to the variable to which the zero position of
   * the driver is to be saved.
   * @return Return Status
   */
  ret_status_t getZeroPosition(float *zero_position);

  /**
   * @brief Set motor phase parameters of the motor in the driver
   *
   * @param phase_resistance phase resistance value of the motor
   * @param phase_inductance phase inductance value of the motor
   * @return Return Status
   */
  ret_status_t setMotorPhaseParameters(float phase_resistance,
                                      float phase_inductance);

  /**
   * @brief Get current angle of the output in degrees
   *
   * @param output_position Pointer to the variable to which the current output
   * angle value is to be saved.
   * @return Return Status
   */
  ret_status_t getOutputPosition(float *output_position);

  /**
   * @brief Get current velocity of the output in degrees per second
   *
   * @param output_velocity Pointer to the variable to which the current
   * velocity value is to be saved.
   * @return Return Status
   */
  ret_status_t getOutputVelocity(float *output_velocity);

  /**
   * @brief Get current acceleration of the output in degrees per second^2
   *
   * @param output_acceleration Pointer to the variable to which the current
   * output acceleration value is to be saved.
   * @return Return Status
   */
  ret_status_t getOutputAcceleration(float *output_acceleration);

  /**
   * @brief Get current torque at the output in Nm
   *
   * @param output_torque Pointer to the variable to which the current output
   * torque value is to be saved.
   * @return Return Status
   */
  ret_status_t getOutputTorque(float *output_torque);

  /**
   * @brief Get current angle of the motor rotor in number of rotations
   *
   * @param motor_position Pointer to the variable to which the current rotation
   * value is to be saved.
   * @return Return Status
   */
  ret_status_t getMotorPosition(float *motor_position);

  /**
   * @brief Get current velocity of the motor rotor in rotations per
   * second[rps]
   *
   * @param motor_velocity Pointer to the variable to which the current velocity
   * value is to be saved.
   * @return Return Status
   */
  ret_status_t getMotorVelocity(float *motor_velocity);

  /**
   * @brief Get current acceleration of the motor rotor in rotations per
   * second^2
   *
   * @param motor_acceleration Pointer to the variable to which the current
   * acceleration value is to be saved.
   * @return Return Status
   */
  ret_status_t getMotorAcceleration(float *motor_acceleration);

  /**
   * @brief Get torque of the motor rotor in Nm
   *
   * @param rotor_torque Pointer to the variable to which the current torque
   * value is to be saved.
   * @return Return Status
   */
  ret_status_t getMotorTorque(float *rotor_torque);

  /**
   * @brief Get motor temperature in degree Celsius. Valid only if a motor
   * thermistor is connected to the driver.
   *
   * @param motor_temperature Pointer to the variable to which the current motor
   * temperature value is to be saved.
   * @return Return Status
   */
  ret_status_t getMotorTemperature(float *motor_temperature);

  /**
   * @brief Get driver temperature in degree Celsius. It uses a thermistor near
   * the FETs to measure the temperature.
   *
   * @param driver_temperature Pointer to the variable to which the current
   * driver temperature value is to be saved.
   * @return Return Status
   */
  ret_status_t getDriverTemperature(float *driver_temperature);

  /**
   * @brief Get motor phase currents in Ampere.
   *
   * @param phA Pointer to the variable to which the current in phase A value is
   * to be saved.
   * @param phB Pointer to the variable to which the current in phase B value is
   * to be saved.
   * @param phC Pointer to the variable to which the current in phase C value is
   * to be saved.
   * @return Return Status
   */
  ret_status_t getMotorPhaseCurrents(float *phA, float *phB, float *phC);

  /**
   * @brief Get motor phase dc calib currents in Ampere.
   *
   * @param dc_calib_phA Pointer to the variable to which the current in phase A value is
   * to be saved.
   * @param dc_calib_phB Pointer to the variable to which the current in phase B value is
   * to be saved.
   * @param dc_calib_phC Pointer to the variable to which the current in phase C value is
   * to be saved.
   * @return Return Status
   */
  ret_status_t getDCCalibPhaseCurrents(float *dc_calib_phA, float *dc_calib_phB, float *dc_calib_phC);

  /**
   * @brief Get the Bus Voltage of the driver
   *
   * @param vbus_voltage Pointer to the variable to which the current bus
   * voltage value is to be saved.
   * @return Return Status
   */
  ret_status_t getBusVoltage(float *vbus_voltage);

  /**
   * @brief Get the Bus Current of the driver
   *
   * @param ibus Pointer to the variable to which the bus current value is to be
   * saved.
   * @return Return Status
   */
  ret_status_t getBusCurrent(float *ibus);

  /**
   * @brief Get the Id and Iq currents of the motor
   *
   * @param id_current Pointer to the variable to which the Id current value is
   * to be saved.
   * @param iq_current Pointer to the variable to which the Iq current value is
   * to be saved.
   * @return Return Status
   */
  ret_status_t getIdqCurrents(float *id_current, float *iq_current);

  /**
   * @brief Function to get the brake current of the driver
   *
   * @param brake_current variable to store the brake current
   * @return Return Status
   */
  ret_status_t getBrakeCurrent(float *brake_current);

  /**
   * @brief Set position controller gain value
   *
   * @param position_gain pos gain value of the position controller
   * @return Return Status
   */
  ret_status_t setPositionControllerGain(float position_gain);

  /**
   * @brief Get position controller gain value
   *
   * @param position_gain Pointer to the variable to which the current position
   * controller gain value is to be saved.
   * @return Return Status
   */
  ret_status_t getPositionControllerGain(float *position_gain);

  /**
   * @brief Set velocity controller gains
   *
   * @param vel_gain velocity gain value of the velocity controller
   * @param vel_integrator_gain velocity integrator gain value of the velocity
   * controller
   *
   * @return Return Status
   */
  ret_status_t setVelocityControllerGains(float vel_gain,
                                          float vel_integrator_gain);

  /**
   * @brief Get controller velocity controller gains
   *
   * @param vel_gain Pointer to the variable to which the velocity gain value of
   * the controller is to be saved.
   * @param vel_integrator_gain Pointer to the variable to which the velocity
   * integrator gain value of the controller is to be saved. controller
   * @return Return Status
   */
  ret_status_t getVelocityControllerGains(float *vel_gain,
                                          float *vel_integrator_gain);

  /**
   * @brief Set current controller bandwith
   *
   * @param bandwidth bandwidth value of the current controller
   *
   * @return Return Status
   */
  ret_status_t setCurrentControllerBandwidth(float bandwidth);

  /**
   * @brief Get current bandwidth of the  controller
   *
   * @param bandwidth Pointer to the variable to which the current controller bandwidth is to be saved.
   * @return Return Status
   */
  ret_status_t getCurrentControllerBandwidth(float *bandwidth);

  /**
   * @brief Get current controller gains
   *
   * @param p_gain Pointer to the variable to which the current controller gain value of
   * the controller is to be saved.
   * @param i_gain Pointer to the variable to which the current controller
   * integrator gain value of the controller is to be saved. controller
   * @return Return Status
   */
  ret_status_t getCurrentControllerGains(float *p_gain,
                                          float *i_gain);

  /**
   * @brief Set motor encoder bandwith
   *
   * @param bandwidth bandwidth value of the encoder
   *
   * @return Return Status
   */
  ret_status_t setMotorEncoderBandwidth(float bandwidth);

  /**
   * @brief Get motor encoder bandwidth of the  controller
   *
   * @param bandwidth Pointer to the variable to which the encoder bandwidth is to be saved.
   * @return Return Status
   */
  ret_status_t getMotorEncoderBandwidth(float *bandwidth);

  /**
   * @brief Set the current data filter value in the controller
   *
   * @param current_filter Value of current filter.
   * @return Return Status
   */
  ret_status_t setCurrentFilter(float current_filter);

  /**
   * @brief Set the current data filter value in the controller
   *
   * @param current_filter Value of current filter.
   * @return Return Status
   */
  ret_status_t getCurrentFilter(float *current_filter);

  /**
   * @brief Set the encoder data filter value in the controller
   *
   * @param encoder_data_filter Value of the encoder data filter.
   * @return Return Status
   */
  ret_status_t setEncoderDataFilter(float encoder_data_filter);

  /**
   * @brief Get the current value of encoder data filter in the controller.
   *
   * @param encoder_data_filter Pointer to the variable to which the current
   * encoder data filter value is to be saved.
   * @return Return Status
   */
  ret_status_t getEncoderDataFilter(float *encoder_data_filter);

  /**
   * @brief Set the encoder bct value of maxxx encoder
   *
   * @param encoder Flag to set the encoder type. False for motor side encoder True for output side encoder
   * @param bct_value Value of the encoder BCT Settings.
   * @return Return Status
   */
  ret_status_t setMAxxxBCTValue(bool encoder, uint8_t bct_value);

  /**
   * @brief Get the encoder bct value of the maxxx encoder.
   * @param encoder Flag to set the encoder type. False for motor side encoder True for output side encoder
   * @param bct_value Pointer to the variable to which the current encoder bct value is to be saved.
   * @return Return Status
   */
  ret_status_t getMAxxxBCTValue(bool encoder, uint8_t *bct_value, bool* status);

    /**
   * @brief Set the encoder trimming settings of maxxxencoder
   *
   * @param encoder Flag to set the encoder type. False for motor side encoder True for output side encoder
   * @param etx Value of the encoder etx settings.
   * @param ety Value of the encoder ety settings.
   * @return Return Status
   */
  ret_status_t setMAxxxTrimmingSettings(bool encoder, bool etx, bool ety);

  /**
   * @brief Get the trimming setting of maxxx encoder.
   * @param encoder Flag to set the encoder type. False for motor side encoder True for output side encoder
   * @param etx Value of the encoder etx settings.
   * @param ety Value of the encoder ety settings.
   * @return Return Status
   */
  ret_status_t getMAxxxTrimmingSettings(bool encoder, bool *etx, bool* ety, bool*status);

    /**
   * @brief Set the encoder filter window of the maxxx encoder
   *
   * @param encoder Flag to set the encoder type. False for motor side encoder True for output side encoder
   * @param etx Value of the encoder etx settings.
   * @param ety Value of the encoder ety settings.
   * @return Return Status
   */
  ret_status_t setMAxxxFilterWindow(bool encoder, uint8_t filter_window);

  /**
   * @brief Get the encoder filter window of maxxx encoder.
   *
   * @param encoder Flag to set the encoder type. False for motor side encoder True for output side encoder
   * @param filter_window Pointer to the variable to which the current
   * encoder filter window is to be saved.
   * @return Return Status
   */
  ret_status_t getMAxxxFilterWindow(bool encoder, uint8_t *filter_window, bool*status);

  /**
   * @brief Get the controller state of the driver
   *
   * @param controller_state Pointer to the variable to which the controller
   * state value is to be saved.
   * @return Return Status
   */
  ret_status_t getControllerState(uint8_t *controller_state);

  /**
   * @brief Get the motor state of the driver
   *
   * @param is_calibrated Pointer to the variable to which the is_calibrated
   * state of motor is to be saved.
   * @param is_armed Pointer to the variable to which the is_armed state of
   * motor is to be saved.
   * @return Return Status
   */
  ret_status_t getMotorState(bool *is_calibrated, bool *is_armed);

  /**
   * @brief Get the state of encoder on the motor side.
   *
   * @param index_found Pointer to the variable to which the index_found state
   * of the motor encoder is to be saved. This value is only valid if the motor
   * side encoder is incremental.
   * @param is_ready Pointer to the variable to which the is_ready state of the
   * motor encoder is to be saved (true means that the motor side encoder is
   * ready)
   * @return Return Status
   */
  ret_status_t getMotorEncoderState(bool *index_found, bool *is_ready);

  /**
   * @brief Get the state of encoder on the output side.
   * @param index_found Pointer to the variable to which the index_found state
   * of the motor encoder is to be saved. This value is only valid if the motor
   * side encoder is incremental.
   * @param is_ready Pointer to the variable to which the is_ready state of the
   * motor encoder is to be saved (true means that the output side encoder is
   * ready)
   * @return Return Status
   */
  ret_status_t getOutputEncoderState(bool *index_found, bool *is_ready);

  /**
   * @brief Get the trajectory done status
   *
   * @param is_done Pointer to the variable to which the status of
   * reaching the trajectory end point is to be saved.
   * @return Return Status
   */
  ret_status_t getTrajectoryDoneStatus(bool *is_done);

  /**
   * @brief Get the Motor Phase Parameters (Phase Resistance and Phase Inductance of the motor windings)
   *
   * @param phase_resistance Pointer to the variable to which the phase resistance of the motor is to be saved.
   * @param phase_inductance Pointer to the variable to which the phase inductance of the motor is to be saved.
   * @return Return Status
   */
  ret_status_t getMotorPhaseParameters(float *phase_resistance, float *phase_inductance);
  /**
   * @brief Get raw encoder data from the encoder on the motor side. Returns the
   * counts in case its an incremental encoder or it returns the raw bit data in
   * case its an absolute encoder.
   *
   * @param encoder_data Pointer to the variable to which the raw encoder value
   * is to be saved.
   * @return Return Status
   */
  ret_status_t getMotorEncoderRawData(int32_t *encoder_data);

  /**
   * @brief Get raw encoder data from the encoder on the Output side. Returns the
   * counts in case its an incremental encoder or it returns the raw bit data in
   * case its an absolute encoder.
   *
   * @param encoder_data Pointer to the variable to which the raw encoder value
   * is to be saved.
   * @return Return Status
   */
  ret_status_t getOutputEncoderRawData(int32_t *encoder_data);

  /**
   * @brief Get CAN communication status
   *
   * @param is_connected_to_master Pointer to the variable to which the status
   * of connection to master node is to be saved.
   * @param is_receiving_heartbeat Pointer to the variable to which the
   * heartbeat receive status is to be saved.
   * @return Return Status
   */
  ret_status_t getCANCommunicationStatus(bool *is_connected_to_master,
                                         bool *is_receiving_heartbeat);

  /**
   * @brief Get controller mode
   *
   * @param control_mode Pointer to the variable to which the value of control
   * mode is to be saved.
   * @return Return Status
   */
  ret_status_t getControllerMode(uint16_t *control_mode);

  /**
   * @brief Get the Driver error status from the driver
   *
   * @param driver_error Pointer to the variable to which the value of driver
   * error is to be saved.
   * @return Return Status
   */
  ret_status_t getDriverFault(uint32_t *driver_error);

  /**
   * @brief Get the error code from the driver
   *
   * @param error_code Pointer to the variable to which the value of error code
   * is to be saved.
   * @return Return Status
   */
  ret_status_t getErrorCode(uint32_t *error_code);

  /**
   * @brief Set the driver in Position Control mode
   *
   * @param angle angle in degrees to where the motor should move
   * @param degrees_per_seconds velocity in degrees per second at which the
   * motor should move
   * @return Return Status
   */
  ret_status_t setPositionControl(float angle, float degrees_per_seconds);

  /**
   * @brief Set the driver in Velocity Control mode
   *
   * @param degrees_per_seconds velocity in degrees per second at which the
   * motor should move.
   * @return Return Status
   */
  ret_status_t setVelocityControl(float degrees_per_seconds);

  /**
   * @brief Set the driver in Torque Control mode
   *
   * @param torque motor torque in Nm
   * @param degrees_per_seconds velocity limit in degrees per seconds
   * @return Return Status
   */
  ret_status_t setTorqueControl(float torque, float degrees_per_seconds);

  /**
   * @brief Set the Driver in Position Control mode with Feedforward parameters
   *
   * @param angle angle in degrees to where the actuator should move.
   * @param velocity_ff feedForward velocity in degrees per second.
   * @param torque_ff feedForward torque in Nm.
   * @return Return Status
   */
  ret_status_t setPositionControlWithFeedForward(float angle, float velocity_ff, float torque_ff);

  /**
   * @brief Set the Driver in Velocity Control mode with FeedForward parameters
   *
   * @param degrees_per_seconds velocity in degrees per second at which the actuator should move.
   * @param torque_ff feedForward torque in Nm.
   * @return Return Status
   */
  ret_status_t setVelocityControlWithFeedForward(float degrees_per_seconds, float torque_ff);

  /**
   * @brief Set the Driver in Trapezoidal Trajectory Control mode
   *
   * @param angle angle in degrees to where the actuator should move
   * @param degrees_per_seconds  velocity in degrees per second at which the actuator should move
   * @param accel_rate  acceleration rate
   * @param decel_rate  deceleration rate
   * @return Return Status
   */
  ret_status_t setTrapezoidalTrajectoryControl(float angle, float degrees_per_seconds,
                                    float accel_rate, float decel_rate);

  /**
   * @brief Set the Driver in S-Curve Trajectory Control mode
   *
   * @param angle angle in degrees to where the actuator should move
   * @param degrees_per_seconds  velocity in degrees per second at which the actuator should move
   * @param accel_rate  acceleration rate
   * @param jerk_rate  jerk rate
   * @return Return Status
   */
  ret_status_t setScurveTrajectoryControl(float angle, float degrees_per_seconds,
                                    float accel_rate, float jerk_rate);

  /**
   * @brief Set the Driver in Velocity Ramp Control mode
   *
   * @param degrees_per_seconds velocity in degrees per second at which the actuator should move
   * @param ramp_rate_degrees_per_seconds velocity ramp rate in degrees per
   * seconds
   * @return Return Status
   */
  ret_status_t setVelocityRampControl(float degrees_per_seconds,
                                      float ramp_rate_degrees_per_seconds);

  /**
   * @brief Get the debug error codes from the driver
   *
   * @param system_error Pointer to the variable to which the value of system error code is saved
   * @param axis_error Pointer to the variable to which the value of axis error code is saved
   * @param motor_error Pointer to the variable to which the value of motor error code is saved
   * @param encoder_error Pointer to the variable to which the value of encoder error code is saved
   * @param controller_error Pointer to the variable to which the value of controller error code is saved
   * @param driver_error Pointer to the variable to which the value of driver error code is saved
   * @return Return Status
   */
  ret_status_t getDebugErrorCode(uint64_t *system_error, uint64_t *axis_error, uint64_t *motor_error, uint64_t *encoder_error, uint64_t *controller_error, uint64_t *driver_error);

  /**
   * @brief Clear driver errors
   *
   * @return Return Status
   */
  ret_status_t clearDriverErrors();

  /**
   * @brief Reboot the driver
   *
   * @return Return Status
   */
  ret_status_t rebootDriver();

  /**
   * @brief Save the configuration to the driver
   *
   * @return Return Status
   */
  ret_status_t saveConfigurations();

  /**
   * @brief Erase Configuration of the driver
   *
   * @return Return Status
   */
  ret_status_t eraseConfigurations();

  /**
   * @brief Enter in DFU Mode
   *
   * @return ret_status_t
   */
  ret_status_t enterDFUMode();

  /**
   * @brief Destroy the Driver object
   *
   */
  ~Driver();

  /**
   * @brief Check if the device is still connected or not
   *
   * @return true
   * @return false
   */
  bool isConnected();

  /**
   * @brief Function to flash a firmware onto the device
   *
   * @param firmware_path path to firmware file
   * @param is_in_dfu_mode flag to set if the driver is already in dfu mode
   * @return ret_status_t
   */
  ret_status_t flash(std::string firmware_path, bool is_in_dfu_mode = false);
};