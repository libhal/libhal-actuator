// Copyright 2026 Malia Labor and the libhal contributors
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

// Use "#pragma once" as an include guard for headers
// This is required because it ensures that the compiler will process this file
// only once, no matter how many times it is included.
#pragma once

#include <cstdint>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/units.hpp>

namespace hal::actuator {
class rx_64
{
public:
  struct config
  {
    hertz baud_rate;
    uint8_t id;
    hal::degrees min_angle = 0;
    hal::degrees max_angle = 300;
  };

  struct angle_range
  {
    hal::degrees min_angle;
    hal::degrees max_angle;
  };

  rx_64(hal::strong_ptr<hal::serial> const& p_serial,
        config const& p_settings,
        hal::strong_ptr<hal::steady_clock> const& p_clock);

  enum class error_type
  {
    input_voltage,
    angle_limit,
    instruction_range,
    checksum,
    current_overload,
    instruction_error,
    no_error
  };

  enum class register_byte : hal::byte
  {
    model_number = 0x00,
    firmware_ver = 0x02,
    id = 0x03,
    baud_rate = 0x04,
    return_delay = 0x05,
    cw_limit = 0x06,
    ccw_limit = 0x08,
    temp_limit = 0x0B,
    min_voltage = 0x0C,
    max_voltage = 0x0D,
    max_torque = 0x0E,
    status_return = 0x10,
    alarm_led = 0x11,
    shutdown = 0x12,
    torque_enable = 0x18,
    led_toggle = 0x19,
    cw_compliance_margin = 0x1A,
    ccw_compliance_margin = 0x1B,
    cw_compliance_slope = 0x1C,
    ccw_compliance_slope = 0x1D,
    goal_position = 0x1E,
    moving_speed = 0x20,
    torque_limit = 0x22,
    present_position = 0x24,
    present_speed = 0x26,
    present_load = 0x28,
    present_voltage = 0x2A,
    present_temp = 0x2B,
    instruction_registered = 0x2C,
    moving_status = 0x2E,
    lock_eeprom = 0x2F,
    punch = 0x30
  };

  bool ping_id(uint8_t p_id);
  void led_toggle(bool p_on);

  bool is_moving();
  std::tuple<float, bool> get_speed();
  float get_voltage();
  uint8_t get_temp();

  float get_torque_limit();
  uint8_t get_torque_enable();
  uint8_t get_temp_limit();
  float get_min_voltage();
  float get_max_voltage();
  hertz get_baud_rate();
  uint16_t get_return_delay_time();
  uint8_t get_id();
  hal::degrees get_min_angle();
  hal::degrees get_max_angle();
  hal::degrees get_current_angle();
  uint16_t get_punch();

  void position(hal::degrees p_angle);
  void set_torque_enable(bool p_enable);
  void set_torque_limit(float p_percent);
  void set_temp_limit(uint8_t p_temp);
  void set_min_voltage(float p_voltage);
  void set_max_voltage(float p_voltage);
  void set_baud_rate(hertz p_baud);
  void set_return_delay_time(uint16_t p_microseconds);
  void set_id(uint8_t p_id);
  void set_min_angle(hal::degrees p_angle);
  void set_max_angle(hal::degrees p_angle);

private:
  void write_small_register(register_byte p_instruction, hal::byte p_value);
  void write_large_register(register_byte p_instruction, uint16_t p_value);

  uint8_t read_small_register(register_byte p_register);
  uint16_t read_large_register(register_byte p_register);

  hal::strong_ptr<hal::serial> m_serial;
  hal::strong_ptr<hal::steady_clock> m_clock;
  hal::byte m_id;
  angle_range m_range;
};
}  // namespace hal::actuator
