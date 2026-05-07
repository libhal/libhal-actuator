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

#include <cmath>

#include <libhal/error.hpp>
#include <libhal/timeout.hpp>
#include <numeric>

#include <libhal-actuator/rx_64.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/units.hpp>

namespace hal::actuator {
rx_64::rx_64(hal::strong_ptr<hal::serial> const& p_serial,
             config const& p_settings,
             hal::strong_ptr<hal::steady_clock> const& p_clock)
  : m_serial(p_serial)
  , m_clock(p_clock)
  , m_id(p_settings.id) {
    // send config instructions
    // write_large_register(register_byte::torque_limit,
    // p_settings.torque_limit); write_small_register(register_byte::temp_limit,
    // p_settings.temp_limit); write_small_register(register_byte::min_voltage,
    // p_settings.min_voltage); write_small_register(register_byte::max_voltage,
    // p_settings.min_voltage); write_large_register(register_byte::cw_limit,
    // p_settings.min_angle); write_large_register(register_byte::ccw_limit,
    // p_settings.max_angle);

    // write_small_register(register_byte::baud_rate, 0x00/*calc byte to send
    // here*/);
    // write_small_register(register_byte::id, m_id);
  };

bool rx_64::ping_id(uint8_t p_id)
{
  using namespace std::chrono_literals;
  std::array<hal::byte, 6> send_bytes = { 0xFF, 0xFF, (hal::byte)p_id,
                                          0x02, 0x01, 0xFB };
  hal::write(*m_serial, send_bytes, hal::never_timeout());
  // try {
  //   auto response = hal::read<6>(*m_serial, hal::create_timeout(*m_clock,
  //   1ms)); if (response[0] == 0xFF && response[1] == 0xFF) {
  //     // device responded
  //     // TODO check full packet and checksum
  //     m_id = p_id;
  //   }
  // } catch (hal::timed_out const&) {
  //   return false;
  // }
  return true;
}

rx_64::error_type rx_64::led_toggle(bool p_on)
{
  hal::byte toggle_byte = 0x00;
  if (p_on) {
    toggle_byte = 0x01;
  }
  write_small_register(register_byte::led_toggle, toggle_byte);
  return error_type::no_error;
}

uint8_t rx_64::read_serial()
{
  using namespace std::chrono_literals;
  uint8_t address = 254;
  try {
    auto response = hal::read<6>(*m_serial, hal::create_timeout(*m_clock, 1ms));
    if (response[0] == 0xFF && response[1] == 0xFF) {
      // device responded
      // TODO check full packet and checksum
      address = response[2];
      m_id = address;
    }
  } catch (hal::timed_out const&) {
    return 254;
  }
  return address;
}

rx_64::error_type rx_64::set_id(uint8_t p_id)
{
  write_small_register(register_byte::id, p_id);
  return error_type::no_error;
}

rx_64::error_type rx_64::write_small_register(register_byte p_register,
                                              hal::byte p_value)
{
  std::array<hal::byte, 8> send_bytes = { 0xFF,    0xFF, m_id,
                                          0x04,    0x03, (hal::byte)p_register,
                                          p_value, 0x00 };
  hal::byte const temp =
    std::accumulate(send_bytes.begin() + 2, send_bytes.begin() + 6, 0);
  // send_bytes[7] = (~temp) & ((1 << 8) - 1);
  send_bytes[7] = ~temp;

  hal::write(*m_serial, send_bytes, hal::never_timeout());
  // std::array<hal::byte, 7> response;
  // hal::read(*m_serial, response, hal::never_timeout());
  return error_type::no_error;
}

rx_64::error_type rx_64::write_large_register(register_byte p_register,
                                              uint16_t p_value)
{
  hal::byte const value_low = p_value;
  hal::byte const value_hi = (p_value >> 8);

  std::array<hal::byte, 9> send_bytes = {
    0xFF,      0xFF,     m_id, 0x04, 0x03, (hal::byte)p_register,
    value_low, value_hi, 0x00
  };
  hal::byte const temp = std::accumulate(&send_bytes[2], &send_bytes[7], 0);
  send_bytes[8] = ~temp;
  hal::write(*m_serial, send_bytes, hal::never_timeout());
  // std::array<hal::byte, 7> response;
  // hal::read(*m_serial, response, hal::never_timeout());
  return error_type::no_error;
}

rx_64::error_type rx_64::position(hal::degrees p_angle)
{
  // 3.41 is angle scale
  auto const angle_bytes = static_cast<u16>(roundf(p_angle * 3.41f));
  return write_large_register(register_byte::goal_position, angle_bytes);
}

}  // namespace hal::actuator
