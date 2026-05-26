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

#include <array>
#include <cstdint>

#include <numeric>
#include <utility>

#include <libhal-util/serial.hpp>
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
    hertz baud_rate = 57600;
    uint8_t id;
    hal::degrees min_angle = 0;
    hal::degrees max_angle = 300;
  };

  rx_64(hal::strong_ptr<hal::serial> const& p_serial,
        config const& p_settings,
        hal::strong_ptr<hal::steady_clock> const& p_clock);

  enum class error_type : u8
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
  void led(bool p_on);

  bool is_moving();
  std::tuple<float, bool> speed();
  float voltage();
  uint8_t temperature();

  float torque_limit();
  uint8_t torque_enable();
  uint8_t temperature_limit();
  float min_voltage();
  float max_voltage();
  hertz baud_rate();
  uint16_t return_delay_time();
  uint8_t id();
  hal::degrees min_angle();
  hal::degrees max_angle();
  hal::degrees position();
  uint16_t punch();
  float moving_speed();

  void position(hal::degrees p_angle);
  void torque_enable(bool p_enable);
  void torque_limit(float p_percent);
  void temperature_limit(uint8_t p_temperature);
  void min_voltage(float p_voltage);
  void max_voltage(float p_voltage);
  void baud_rate(hertz p_baud);
  void return_delay_time(uint16_t p_microseconds);
  void id(uint8_t p_id);
  void min_angle(hal::degrees p_angle);
  void max_angle(hal::degrees p_angle);
  void speed(float p_rpms);

  void sync_position(hal::degrees p_angle, rx_64 p_opposing_servo);

private:
  template<usize Size>
  auto read_register(register_byte p_register_address)
  {
    using namespace std::chrono_literals;

    std::array<hal::byte, 8> send_bytes = { 0xFF,
                                            0xFF,
                                            m_id,
                                            0x04,
                                            0x02,
                                            (hal::byte)p_register_address,
                                            (hal::byte)Size,
                                            0x00 };
    hal::byte const checksum =
      std::accumulate(&send_bytes[2], &send_bytes[7], 0);
    send_bytes[7] = ~checksum;
    hal::write(*m_serial, send_bytes, hal::never_timeout());
    std::array<hal::byte, Size> return_array{};

    try {
      auto constexpr read_size = 6 + Size;

      auto response =
        hal::read<read_size>(*m_serial, hal::create_timeout(*m_clock, 500ms));
      if (response[0] == 0xFF && response[1] == 0xFF) {
        // device responded
        // TODO: checksum not working, fix later
        // hal::byte calculated_chksm =
        //   std::accumulate(&response[2], &response[read_size - 2], 0);
        // calculated_chksm = ~calculated_chksm;
        // if (calculated_chksm == response[read_size - 1]) {
        for (usize i = 0; i < Size; i++) {
          return_array[i] = response[5 + i];
        }
        // }
        return return_array;
      }
    } catch (hal::timed_out const&) {
      return return_array;
    }
    return return_array;
  }

  template<usize Size>
  void write_register(register_byte p_register_address,
                      std::array<hal::byte, Size> p_data)
  {
    auto constexpr send_data_size = 7 + Size;
    hal::byte packet_length = 0x03 + Size;
    std::array<hal::byte, send_data_size> send_bytes = {
      0xFF, 0xFF, m_id, packet_length, 0x03, (hal::byte)p_register_address
    };
    for (uint8_t i = 0; i < p_data.size(); i++) {
      send_bytes[6 + i] = p_data[i];
    }
    hal::byte const checksum =
      std::accumulate(&send_bytes[2], &send_bytes[send_data_size - 1], 0);
    send_bytes[send_data_size - 1] = ~checksum;

    hal::write(*m_serial, send_bytes, hal::never_timeout());

    try {
      using namespace std::chrono_literals;
      auto const response =
        hal::read<6>(*m_serial, hal::create_timeout(*m_clock, 500ms));
      if (response[0] == 0xFF && response[1] == 0xFF) {
        // device responded
        hal::byte received_chksm =
          std::accumulate(&response[2], &response[4], 0);
        received_chksm = ~received_chksm;
        if (received_chksm == response[5]) {
          // checksum match
        }
      }
    } catch (hal::timed_out const&) {
      return;
    }
  }

  hal::strong_ptr<hal::serial> m_serial;
  hal::strong_ptr<hal::steady_clock> m_clock;
  hal::byte m_id;
  std::pair<hal::degrees, hal::degrees> m_range;
};
}  // namespace hal::actuator
