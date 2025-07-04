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

#include <libhal-util/can.hpp>
#include <libhal/angular_velocity_sensor.hpp>
#include <libhal/can.hpp>
#include <libhal/motor.hpp>
#include <libhal/rotation_sensor.hpp>
#include <libhal/servo.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/temperature_sensor.hpp>
#include <libhal/units.hpp>

namespace hal::actuator {
/**
 * @brief Driver for RMD motors equip with the DRC motor drivers
 *
 */
class rmd_drc_v2
{
public:
  /// Commands that can be issued to a RMD-X motor
  enum class read : hal::byte
  {
    /**
     * @brief Status1 + error flag information read request command
     *
     * Sending this request will update the following fields in the feedback_t:
     *
     *    - raw_multi_turn_angle
     */
    multi_turns_angle = 0x92,
    /**
     * @brief Status1 + error flag information read request command
     *
     * Sending this request will update the following fields in the feedback_t:
     *
     *    - raw_motor_temperature
     *    - over_voltage_protection_tripped
     *    - over_temperature_protection_tripped
     */
    status_1_and_error_flags = 0x9A,
    /**
     * @brief Status2 read request command
     *
     * Sending this request will update the following fields in the feedback_t:
     *
     *    - raw_motor_temperature
     *    - raw_current
     *    - raw_speed
     *    - encoder
     */
    status_2 = 0x9C,
  };

  /// Commands for actuate the motor
  enum class actuate : hal::byte
  {
    speed = 0xA2,
    position_2 = 0xA4,
  };

  /// Commands for updating motor configuration data
  enum class write : hal::byte
  {
    pid_to_ram = 0x31,
    pid_to_rom = 0x32,
    acceleration_data_to_ram = 0x34,
    encoder_offset = 0x91,
    current_position_to_rom_as_motor_zero = 0x19,
  };

  /// Commands for controlling the motor as a whole
  enum class system : hal::byte
  {
    clear_error_flag = 0x9B,
    off = 0x80,
    stop = 0x81,
    running = 0x88,
  };

  /// Structure containing all of the forms of feedback acquired by an RMD-X
  /// motor
  struct feedback_t
  {
    /// Every time a message from our motor is received this number increments.
    /// This can be used to indicate if the feedback has updated since the last
    /// time it was read.
    std::uint32_t message_number = 0;
    /// Raw multi-turn angle (0.01°/LSB)
    std::int64_t raw_multi_turn_angle{ 0 };
    /// Current flowing through the motor windings
    /// (-2048 <-> 2048 ==> -33A <-> 33A)
    std::int16_t raw_current{ 0 };
    /// Rotational velocity of the motor (1 degrees per second (dps)/LSB)
    std::int16_t raw_speed{ 0 };
    /// Motor's supply voltage (0.1V/LSB)
    std::int16_t raw_volts{ 0 };
    /// Signed 16-bit raw encoder count value of the motor
    std::int16_t encoder{ 0 };
    /// Core temperature of the motor (1C/LSB)
    std::int8_t raw_motor_temperature{ 0 };
    /// 8-bit value containing error flag information
    std::uint8_t raw_error_state{ 0 };

    [[nodiscard]] hal::ampere current() const noexcept;
    [[nodiscard]] hal::rpm speed() const noexcept;
    [[nodiscard]] hal::volts volts() const noexcept;
    [[nodiscard]] hal::celsius temperature() const noexcept;
    [[nodiscard]] hal::degrees angle() const noexcept;

    /**
     * @brief Return if the motor has detected an over voltage event
     *
     * In order for this field to be updated a feedback_request with
     * status_1_and_error_flags must be issued.
     *
     * @return true - over voltage protection tripped
     * @return false - over voltage protection has not tripped
     */
    [[nodiscard]] bool over_voltage_protection_tripped() const noexcept;

    /**
     * @brief Return if the motor has detected an over temperature event
     *
     * In order for this field to be updated a feedback_request with
     * status_1_and_error_flags must be issued.
     *
     * @return true - over temperature protection tripped
     * @return false - over temperature protection has not tripped
     */
    [[nodiscard]] bool over_temperature_protection_tripped() const noexcept;
  };

  /**
   * @brief Rotation sensor adaptor for DRC motors
   *
   */
  class rotation_sensor : public hal::rotation_sensor
  {
  public:
    rotation_sensor(rotation_sensor const&) = delete;
    rotation_sensor& operator=(rotation_sensor const&) = delete;
    rotation_sensor(rotation_sensor&&) = default;
    rotation_sensor& operator=(rotation_sensor&&) = default;
    ~rotation_sensor() override = default;

  private:
    friend class rmd_drc_v2;
    rotation_sensor(rmd_drc_v2& p_drc);
    hal::rotation_sensor::read_t driver_read() override;
    rmd_drc_v2* m_drc = nullptr;
  };

  /**
   * @brief Temperature sensor adaptor for DRC motors
   *
   */
  class temperature_sensor : public hal::temperature_sensor
  {
  public:
    temperature_sensor(temperature_sensor const&) = delete;
    temperature_sensor& operator=(temperature_sensor const&) = delete;
    temperature_sensor(temperature_sensor&&) = default;
    temperature_sensor& operator=(temperature_sensor&&) = default;
    ~temperature_sensor() override = default;

  private:
    friend class rmd_drc_v2;
    temperature_sensor(rmd_drc_v2& p_drc);
    celsius driver_read() override;
    rmd_drc_v2* m_drc = nullptr;
  };

  /**
   * @brief Motor interface adaptor for DRC
   *
   */
  class motor : public hal::motor
  {
  public:
    motor(motor const&) = delete;
    motor& operator=(motor const&) = delete;
    motor(motor&&) = default;
    motor& operator=(motor&&) = default;
    ~motor() override = default;

  private:
    friend class rmd_drc_v2;
    motor(rmd_drc_v2& p_drc, hal::rpm p_max_speed);
    void driver_power(float p_power) override;

    rmd_drc_v2* m_drc = nullptr;
    hal::rpm m_max_speed;
  };

  /**
   * @brief Servo interface adaptor for DRC
   *
   */
  class servo : public hal::servo
  {
  public:
    servo(servo const&) = delete;
    servo& operator=(servo const&) = delete;
    servo(servo&&) = default;
    servo& operator=(servo&&) = default;
    ~servo() override = default;

  private:
    friend class rmd_drc_v2;
    servo(rmd_drc_v2& p_drc, hal::rpm p_max_speed);
    void driver_position(hal::degrees p_position) override;
    rmd_drc_v2* m_drc = nullptr;
    hal::rpm m_max_speed;
  };

  /**
   * @brief angular velocity sensor adaptor for DRC
   *
   */
  class angular_velocity_sensor : public hal::angular_velocity_sensor
  {
  public:
    angular_velocity_sensor(angular_velocity_sensor const&) = delete;
    angular_velocity_sensor& operator=(angular_velocity_sensor const&) = delete;
    angular_velocity_sensor(angular_velocity_sensor&&) = default;
    angular_velocity_sensor& operator=(angular_velocity_sensor&&) = default;
    ~angular_velocity_sensor() override = default;

  private:
    friend class rmd_drc_v2;
    angular_velocity_sensor(rmd_drc_v2& p_drc);
    hal::rpm driver_read() override;
    rmd_drc_v2* m_drc = nullptr;
  };

  /**
   * @brief Create a new device driver drc
   *
   * This factory function will power cycle the motor
   *
   * @param p_transceiver - The can transceiver connected to the bus with the
   * servo present.
   * @param p_filter - Identifier filter to allow responses from the servo to be
   * received by the device.
   * @param p_device_id - The CAN ID of the motor
   * @param p_clock - clocked used to determine timeouts
   * @param p_gear_ratio - gear ratio of the motor
   * @param p_max_response_time - maximum amount of time to wait for a response
   * from the motor.
   * @throws hal::timed_out - if the p_max_response_time is exceeded
   */
  rmd_drc_v2(
    hal::can_transceiver& p_transceiver,
    hal::can_identifier_filter& p_filter,
    hal::steady_clock& p_clock,
    float p_gear_ratio,
    hal::u32 p_device_id,
    hal::time_duration p_max_response_time = std::chrono::milliseconds(10));

  rmd_drc_v2(rmd_drc_v2&) = delete;
  rmd_drc_v2& operator=(rmd_drc_v2&) = delete;
  rmd_drc_v2(rmd_drc_v2&&) noexcept = delete;
  rmd_drc_v2& operator=(rmd_drc_v2&&) noexcept = delete;
  ~rmd_drc_v2() = default;

  /**
   * @brief Create a hal::rotation_sensor driver using the drc driver
   *
   * @return drc_rotation_sensor - motor implementation based on the drc
   * driver. This object's lifetime must exceed the lifetime of the returned
   * object.
   */
  rotation_sensor acquire_rotation_sensor();

  /**
   * @brief Create a hal::temperature_sensor driver using the drc driver
   *
   * @return drc_temperature_sensor - temperature sensor implementation based on
   * the drc driver. This object's lifetime must exceed the lifetime of the
   * returned object.
   */
  temperature_sensor acquire_temperature_sensor();

  /**
   * @brief Create a hal::motor implementation from the drc driver
   *
   * @param p_max_speed - maximum speed of the motor represented by +1.0 and
   * -1.0
   * @return rmd_drc_v2::motor - motor implementation based on the drc driver.
   * This object's lifetime must exceed the lifetime of the returned object.
   */
  motor acquire_motor(hal::rpm p_max_speed);

  /**
   * @brief Create a hal::servo driver using the drc driver
   *
   * @param p_max_speed - maximum speed of the servo when moving to an angle
   * @return drc_servo - servo implementation based on the drc driver. This
   * object's lifetime must exceed the lifetime of the returned object.
   */
  servo acquire_servo(hal::rpm p_max_speed);

  /**
   * @brief Create a hal::angular_velocity_sensor driver using the drc driver
   *
   * @return angular_velocity_sensor - angular_velocity_sensor implementation
   * based on the drc driver. This object's lifetime must exceed the lifetime of
   * the returned object.
   */
  angular_velocity_sensor acquire_angular_velocity_sensor();

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
   * @param p_speed - maximum speed in rpm's
   * @throws hal::timed_out - if a response is not returned within the max
   * response time set at creation.
   */
  void position_control(degrees p_angle, rpm p_speed);

  /**
   * @brief Send system control commands to the device
   *
   * @param p_system_command - system control command to send to the device
   * status.
   * @throws hal::timed_out - if a response is not returned within the max
   * response time set at creation.
   */
  void system_control(system p_system_command);

  /**
   * @brief Returns a reference to the internal feedback
   *
   * @return feedback_t const&
   */
  [[nodiscard]] feedback_t const& feedback() const;

private:
  void handle_message(can_message const& p_message);
  /**
   * @brief Send command on can bus to the motor using its device ID
   *
   * @param p_payload - command data to be sent to the device
   */
  void send(std::array<hal::byte, 8> p_payload);

  feedback_t m_feedback{};
  hal::can_message_finder m_can;
  hal::steady_clock* m_clock;
  float m_gear_ratio;
  hal::time_duration m_max_response_time;
};
}  // namespace hal::actuator
