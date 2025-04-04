// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>

#include <libhal-canrouter/can_router.hpp>
#include <libhal/can.hpp>
#include <libhal/current_sensor.hpp>
#include <libhal/motor.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/servo.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/temperature_sensor.hpp>
#include <libhal/units.hpp>

namespace hal::actuator {
/**
 * @brief Driver for RMD series motors equip with the MC-X motor driver
 *
 */
class rmd_mc_x
{
public:
  /// Commands that can be issued to a RMD-X motor
  enum class read : hal::byte
  {
    multi_turns_angle = 0x92,
    status_1_and_error_flags = 0x9A,
    status_2 = 0x9C,
  };

  /// Commands for actuate the motor
  enum class actuate : hal::byte
  {
    torque = 0xA1,
    speed = 0xA2,
    position = 0xA5,
  };

  /// Commands for updating motor configuration data
  enum class write : hal::byte
  {
    // None supported currently
  };

  /// Commands for controlling the motor as a whole
  enum class system : hal::byte
  {
    off = 0x80,
    stop = 0x81,
  };

  /// Structure containing all of the forms of feedback acquired by an RMD-X
  /// motor
  struct feedback_t
  {
    /// Every time a message from our motor is received this number increments.
    /// This can be used to indicate if the feedback has updated since the last
    /// time it was read.
    std::uint32_t message_number = 0;
    /// Represents the multi-turn absolute angle of the encoder relative to its
    /// zero starting point (0.01°/LSB)
    std::int64_t raw_multi_turn_angle{ 0 };
    /// 16-bit value containing error flag information
    std::uint16_t raw_error_state{ 0 };
    /// Current flowing through the motor windings
    ///  (-2048 <-> 2048 ==> -33A <-> 33A)
    std::int16_t raw_current{ 0 };
    /// Rotational velocity of the motor (1 degrees per second (dps)/LSB)
    std::int16_t raw_speed{ 0 };
    /// Motor's supply voltage (0.1V/LSB)
    std::int16_t raw_volts{ 0 };
    /// Signed 16-bit raw encoder count value of the motor
    std::int16_t encoder{ 0 };
    /// Core temperature of the motor (1C/LSB)
    std::int8_t raw_motor_temperature{ 0 };

    hal::ampere current() const noexcept;
    hal::rpm speed() const noexcept;
    hal::volts volts() const noexcept;
    hal::celsius temperature() const noexcept;
    hal::degrees angle() const noexcept;
    bool motor_stall() const noexcept;
    bool low_pressure() const noexcept;
    bool over_voltage() const noexcept;
    bool over_current() const noexcept;
    bool power_overrun() const noexcept;
    bool speeding() const noexcept;
    bool over_temperature() const noexcept;
    bool encoder_calibration_error() const noexcept;
  };

  /**
   * @brief Create a new mc_x device driver
   *
   * @param p_router - can router to use
   * @param p_clock - clocked used to determine timeouts
   * @param p_gear_ratio - gear ratio of the motor
   * @param p_device_id - The CAN ID of the motor
   * @param p_max_response_time - maximum amount of time to wait for a response
   * from the motor.
   * @throws hal::timed_out - if the p_max_response_time is exceeded
   */
  rmd_mc_x(
    hal::can_router& p_router,
    hal::steady_clock& p_clock,
    float p_gear_ratio,
    can::id_t p_device_id,
    hal::time_duration p_max_response_time = std::chrono::milliseconds(10));

  rmd_mc_x(rmd_mc_x&) = delete;
  rmd_mc_x& operator=(rmd_mc_x&) = delete;
  rmd_mc_x(rmd_mc_x&&) noexcept;
  rmd_mc_x& operator=(rmd_mc_x&&) noexcept;

  /**
   * @brief Get feedback about the motor
   *
   * This object contains cached data from each response returned from the
   * motor. It is updated when any of the control or feedback APIs are called.
   * This object will not update without one of those APIs being called.
   *
   * @return const feedback_t& - information about the motor
   * @throws hal::timed_out - if a response is not returned within the max
   * response time set at creation.
   */
  feedback_t const& feedback() const;

  /**
   * @brief Request feedback from the motor
   *
   * @param p_command - the request to command the motor to respond with
   * @throws hal::timed_out - if a response is not returned within the max
   * response time set at creation.
   */
  void feedback_request(read p_command);

  /**
   * @brief Rotate motor shaft at the designated speed
   *
   * @param p_speed - speed in rpm to move the motor shaft at. Positive values
   * rotate the motor shaft clockwise, negative values rotate the motor shaft
   * counter-clockwise assuming you are looking directly at the motor shaft.
   * @throws hal::timed_out - if a response is not returned within the max
   * response time set at creation.
   */
  void velocity_control(rpm p_speed);

  /**
   * @brief Move motor shaft to a specific angle
   *
   * @param p_angle - angle position in degrees to move to
   * @param speed - speed in rpm's
   * @throws hal::timed_out - if a response is not returned within the max
   * response time set at creation.
   */
  void position_control(degrees p_angle, rpm speed);

  /**
   * @brief Send system control commands to the device
   *
   * @param p_system_command - system control command to send to the device
   * status
   * @throws hal::timed_out - if a response is not returned within the max
   * response time set at creation.
   */
  void system_control(system p_system_command);

  /**
   * @brief Handle messages from the can bus with this devices ID
   *
   * Meant mostly for testing purposes.
   *
   * @param p_message - message received from the bus
   */
  void operator()(can::message_t const& p_message);

private:
  /**
   * @brief Send command on can bus to the motor using its device ID
   *
   * @param p_payload - command data to be sent to the device
   */
  void send(std::array<hal::byte, 8> p_payload);

  feedback_t m_feedback{};
  hal::steady_clock* m_clock;
  hal::can_router* m_router;
  hal::can_router::route_item m_route_item;
  float m_gear_ratio;
  can::id_t m_device_id;
  hal::time_duration m_max_response_time;
};

/**
 * @brief Control a mc_x motor driver like a hal::motor
 *
 */
class rmd_mc_x_motor : public hal::motor
{
private:
  rmd_mc_x_motor(rmd_mc_x& p_mc_x, hal::rpm p_max_speed);
  void driver_power(float p_power) override;
  friend rmd_mc_x_motor make_motor(rmd_mc_x& p_mc_x, hal::rpm p_max_speed);
  rmd_mc_x* m_mc_x = nullptr;
  hal::rpm m_max_speed;
};

/**
 * @brief Create a hal::motor driver using the MC-X driver
 *
 * @param p_mc_x - reference to a MC-X driver. This object's lifetime must
 * exceed the lifetime of the returned object.
 * @param p_max_speed - maximum speed of the motor represented by +1.0 and -1.0
 * @return mc_x_motor - motor implementation using the MC-X driver
 */
rmd_mc_x_motor make_motor(rmd_mc_x& p_mc_x, hal::rpm p_max_speed);

/**
 * @brief Reports the rotation of the DRC motor
 *
 */
class rmd_mc_x_rotation : public hal::rotation_sensor
{
private:
  rmd_mc_x_rotation(rmd_mc_x& p_mc_x);
  hal::rotation_sensor::read_t driver_read() override;
  friend rmd_mc_x_rotation make_rotation_sensor(rmd_mc_x& p_mc_x);
  rmd_mc_x* m_mc_x = nullptr;
};

/**
 * @brief Create a hal::rotation_sensor driver using the MC-X driver
 *
 * @param p_mc_x - reference to a MC-X driver. This object's lifetime must
 * exceed the lifetime of the returned object.
 * @return mc_x_rotation - rotation sensor implementation based on the
 * MC-X driver
 */
rmd_mc_x_rotation make_rotation_sensor(rmd_mc_x& p_mc_x);

/**
 * @brief Control a mc_x motor driver like a hal::servo
 *
 */
class rmd_mc_x_servo : public hal::servo
{
private:
  rmd_mc_x_servo(rmd_mc_x& p_mc_x, hal::rpm p_max_speed);
  void driver_position(hal::degrees p_position) override;
  friend rmd_mc_x_servo make_servo(rmd_mc_x& p_mc_x, hal::rpm p_max_speed);
  rmd_mc_x* m_mc_x = nullptr;
  hal::rpm m_max_speed;
};

/**
 * @brief Create a hal::rotation_sensor driver using the MC-X driver
 *
 * @param p_mc_x - reference to a MC-X driver. This object's lifetime must
 * exceed the lifetime of the returned object.
 * @param p_max_speed - maximum speed of the motor when moving to an angle
 * @return mc_x_rotation - rotation sensor implementation based on the
 * MC-X driver
 */
rmd_mc_x_servo make_servo(rmd_mc_x& p_mc_x, hal::rpm p_max_speed);

/**
 * @brief Reports the temperature of the DRC motor
 *
 */
class rmd_mc_x_temperature : public hal::temperature_sensor
{
private:
  rmd_mc_x_temperature(rmd_mc_x& p_mc_x);
  hal::celsius driver_read() override;
  friend rmd_mc_x_temperature make_temperature_sensor(rmd_mc_x& p_mc_x);
  rmd_mc_x* m_mc_x = nullptr;
};

/**
 * @brief Create a hal::temperature_sensor driver using the MC-X driver
 *
 * @param p_mc_x - reference to a MC-X driver. This object's lifetime must
 * exceed the lifetime of the returned object.
 * @return mc_x_temperature - temperature sensor implementation based on
 * the MC-X driver
 */
rmd_mc_x_temperature make_temperature_sensor(rmd_mc_x& p_mc_x);

/**
 * @brief current sensor adaptor for mc_x
 *
 */
class rmd_mc_x_current_sensor : public hal::current_sensor
{
private:
  rmd_mc_x_current_sensor(rmd_mc_x& p_mc_x);
  hal::ampere driver_read() override;
  friend rmd_mc_x_current_sensor make_current_sensor(rmd_mc_x& p_mc_x);
  rmd_mc_x* m_mc_x = nullptr;
};

/**
 * @brief Create a hal::current_sensor driver using the mc_x driver
 *
 * @param p_mc_x - reference to a mc_x driver. This object's lifetime must
 * exceed the lifetime of the returned object
 * @return mc_x_current_sensor - current_sensor implementation based on the mc_x
 * driver
 */
rmd_mc_x_current_sensor make_current_sensor(rmd_mc_x& p_mc_x);
}  // namespace hal::actuator

// NOTE: Inject make functions into `hal` namespace
namespace hal {
using actuator::make_current_sensor;
using actuator::make_motor;
using actuator::make_rotation_sensor;
using actuator::make_servo;
using actuator::make_temperature_sensor;
}  // namespace hal
