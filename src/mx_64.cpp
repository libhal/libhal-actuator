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

#include <bitset>
#include <cmath>

#include <algorithm>
#include <cstdint>
#include <libhal/error.hpp>
#include <libhal/timeout.hpp>
#include <numeric>

#include <libhal-actuator/mx_64.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/units.hpp>

namespace hal::actuator {
mx_64::mx_64(hal::strong_ptr<hal::serial> const& p_serial,
             config const& p_settings,
             hal::strong_ptr<hal::steady_clock> const& p_clock)
  : m_serial(p_serial)
  , m_clock(p_clock)
  , m_id(p_settings.id)
{
  using namespace std::chrono_literals;
  m_serial->configure({ .baud_rate = p_settings.baud_rate });
  hal::delay(*m_clock, 10ms);
  set_min_angle(p_settings.min_angle);
  hal::delay(*m_clock, 10ms);
  set_max_angle(p_settings.max_angle);
  hal::delay(*m_clock, 10ms);
  set_punch(0x0020);
};

bool mx_64::ping_id(uint8_t p_id)
{
  using namespace std::chrono_literals;
  std::array<hal::byte, 6> send_bytes = { 0xFF, 0xFF, (hal::byte)p_id,
                                          0x02, 0x01, 0x00 };
  hal::byte const temp = std::accumulate(&send_bytes[2], &send_bytes[5], 0);
  send_bytes[5] = ~temp;
  hal::write(*m_serial, send_bytes, hal::never_timeout());

  try {
    auto response =
      hal::read<6>(*m_serial, hal::create_timeout(*m_clock, 500ms));
    if (response[0] == 0xFF && response[1] == 0xFF) {
      // device responded
      // TODO check full packet and checksum
      m_id = p_id;
    }
  } catch (hal::timed_out const&) {
    return false;
  }
  return true;
}

void mx_64::led_toggle(bool p_on)
{
  hal::byte toggle_byte = 0x00;
  if (p_on) {
    toggle_byte = 0x01;
  }
  write_small_register(register_byte::led_toggle, toggle_byte);
}

bool mx_64::is_moving()
{
  auto response = read_small_register(register_byte::moving_status);
  if (response == 0x01) {
    return true;
  }
  return false;
}

std::tuple<float, bool> mx_64::get_speed()
{
  auto response = read_large_register(register_byte::present_speed);
  std::bitset<16> bits{ response };
  bool clockwise = bits[9];  // 10th bit is direction
  bits.set(9, false);
  float rpms = static_cast<float>(response) * 0.111f;
  return { rpms, clockwise };
}

float mx_64::get_voltage()
{
  auto response = read_small_register(register_byte::present_voltage);
  return (static_cast<float>(response) / 10);
}

uint8_t mx_64::get_temp()
{
  return read_small_register(register_byte::present_temp);
}

float mx_64::get_torque_limit()
{
  auto response = read_large_register(register_byte::torque_limit);
  return (static_cast<float>(response) / 1023);
}

uint8_t mx_64::get_torque_enable()
{
  return read_small_register(mx_64::register_byte::torque_enable);
}

uint8_t mx_64::get_temp_limit()
{
  return read_small_register(register_byte::temp_limit);
}

float mx_64::get_min_voltage()
{
  auto response = read_small_register(register_byte::min_voltage);
  return (static_cast<float>(response) / 10);
}

float mx_64::get_max_voltage()
{
  auto response = read_small_register(register_byte::max_voltage);
  return (static_cast<float>(response) / 10);
}

hertz mx_64::get_baud_rate()
{
  auto response = read_small_register(register_byte::baud_rate);
  return (static_cast<hertz>(2000000) / static_cast<hertz>(response + 1));
}

uint16_t mx_64::get_return_delay_time()
{
  auto response = read_small_register(register_byte::return_delay);
  return (response * 2);
}

uint8_t mx_64::get_id()
{
  return m_id;
}

hal::degrees mx_64::get_min_angle()
{
  auto angle_byte = read_large_register(register_byte::cw_limit);

  return static_cast<u16>(
    hal::map(angle_byte, std::make_pair(0, 4095), std::make_pair(0, 360)));
}

hal::degrees mx_64::get_max_angle()
{
  auto angle_byte = read_large_register(register_byte::ccw_limit);
  return static_cast<u16>(
    hal::map(angle_byte, std::make_pair(0, 4095), std::make_pair(0, 360)));
}

hal::degrees mx_64::get_current_angle()
{
  auto angle_byte = read_large_register(register_byte::present_position);
  return static_cast<u16>(
    hal::map(angle_byte, std::make_pair(0, 4095), std::make_pair(0, 360)));
}

uint16_t mx_64::get_punch()
{
  return read_large_register(mx_64::register_byte::punch);
}

bool mx_64::get_torque_ctrl_mode()
{
  auto response = read_small_register(register_byte::torque_ctrl_mode_enable);
  if (response == 0x01) {
    return true;
  }
  return false;
}

void mx_64::position(hal::degrees p_angle)
{
  auto clamped_angle =
    std::clamp(p_angle, m_range.min_angle, m_range.max_angle);

  auto angle_byte = static_cast<u16>(clamped_angle * 11.375f);

  // auto angle_byte = static_cast<u16>(hal::map(
  //   clamped_angle, std::make_pair(0.0, 360.0), std::make_pair(0.0, 4095.0)));
  write_large_register(register_byte::goal_position, angle_byte);
}

void mx_64::set_torque_enable(bool p_enable)
{
  hal::byte data_byte = 0x00;
  if (p_enable) {
    data_byte = 0x01;
  }
  write_small_register(register_byte::torque_enable, data_byte);
}

void mx_64::set_torque_limit(float p_percent)
{
  auto clamped_percent = std::clamp(p_percent, 0.0f, 100.0f);
  auto value = static_cast<uint16_t>(1023 * (clamped_percent / 100));
  write_large_register(register_byte::torque_limit, value);
}

void mx_64::set_temp_limit(uint8_t p_temp)
{
  auto clamped_temp = std::clamp(p_temp, (uint8_t)0, (uint8_t)100);
  write_small_register(register_byte::temp_limit, clamped_temp);
}

void mx_64::set_min_voltage(float p_voltage)
{
  auto clamped_volt = std::clamp(p_voltage, 5.0f, 25.0f);
  auto value = static_cast<uint16_t>(clamped_volt * 10);
  write_small_register(register_byte::min_voltage, value);
}

void mx_64::set_max_voltage(float p_voltage)
{
  auto clamped_volt = std::clamp(p_voltage, 5.0f, 25.0f);
  auto value = static_cast<uint16_t>(clamped_volt * 10);
  write_small_register(register_byte::max_voltage, value);
}

// void mx_64::set_baud_rate(hertz p_baud)
// {
// }

void mx_64::set_return_delay_time(uint16_t p_microseconds)
{
  uint8_t value = (p_microseconds / 2);
  write_small_register(register_byte::return_delay, value);
}

void mx_64::set_id(uint8_t p_id)
{
  m_id = p_id;
  write_small_register(register_byte::id, p_id);
}

void mx_64::set_min_angle(hal::degrees p_angle)
{
  m_range.min_angle = std::clamp(p_angle, 0.0f, 360.0f);

  auto angle_byte = static_cast<u16>(m_range.min_angle * 11.375f);

  // auto angle_byte = static_cast<u16>(hal::map(
  //   m_range.min_angle, std::make_pair(0, 360), std::make_pair(0, 4095)));
  write_large_register(register_byte::cw_limit, angle_byte);
}

void mx_64::set_max_angle(hal::degrees p_angle)
{
  m_range.max_angle = std::clamp(p_angle, 0.0f, 360.0f);

  auto angle_byte = static_cast<u16>(m_range.max_angle * 11.375f);
  // auto angle_byte = static_cast<u16>(hal::map(
  //   m_range.max_angle, std::make_pair(0, 360), std::make_pair(0, 4095)));
  write_large_register(register_byte::ccw_limit, angle_byte);
}

void mx_64::set_punch(uint16_t p_value)
{
  write_large_register(register_byte::punch, p_value);
}

void mx_64::write_small_register(register_byte p_register, hal::byte p_value)
{
  std::array<hal::byte, 8> send_bytes = { 0xFF,    0xFF, m_id,
                                          0x04,    0x03, (hal::byte)p_register,
                                          p_value, 0x00 };
  hal::byte const temp = std::accumulate(&send_bytes[2], &send_bytes[7], 0);
  send_bytes[7] = ~temp;

  hal::write(*m_serial, send_bytes, hal::never_timeout());

  try {
    using namespace std::chrono_literals;
    auto response =
      hal::read<6>(*m_serial, hal::create_timeout(*m_clock, 500ms));
    if (response[0] == 0xFF && response[1] == 0xFF) {
      // device responded
    }
  } catch (hal::timed_out const&) {
    return;
  }
}

void mx_64::write_large_register(register_byte p_register, uint16_t p_value)
{
  hal::byte const value_low = p_value;
  hal::byte const value_hi = (p_value >> 8);

  std::array<hal::byte, 9> send_bytes = {
    0xFF,      0xFF,     m_id, 0x05, 0x03, (hal::byte)p_register,
    value_low, value_hi, 0x00
  };
  hal::byte const temp = std::accumulate(&send_bytes[2], &send_bytes[8], 0);
  send_bytes[8] = ~temp;

  bool status_packet_received = false;
  while (status_packet_received == false) {
    hal::write(*m_serial, send_bytes, hal::never_timeout());
    try {
      using namespace std::chrono_literals;
      auto response =
        hal::read<6>(*m_serial, hal::create_timeout(*m_clock, 100ms));
      if (response[0] == 0xFF && response[1] == 0xFF) {
        // device responded
        status_packet_received = true;
      }
    } catch (hal::timed_out const&) {
      // try again
      status_packet_received = false;
    }
  }
}

uint8_t mx_64::read_small_register(mx_64::register_byte p_register)
{
  using namespace std::chrono_literals;

  std::array<hal::byte, 8> send_bytes = { 0xFF, 0xFF, m_id,
                                          0x04, 0x02, (hal::byte)p_register,
                                          0x01, 0x00 };
  hal::byte const temp = std::accumulate(&send_bytes[2], &send_bytes[7], 0);
  send_bytes[7] = ~temp;
  hal::write(*m_serial, send_bytes, hal::never_timeout());

  try {
    auto response =
      hal::read<7>(*m_serial, hal::create_timeout(*m_clock, 100ms));
    if (response[0] == 0xFF && response[1] == 0xFF) {
      // device responded
      // TODO check full packet and checksum
      return response[5];
    }
  } catch (hal::timed_out const&) {
    return 0;
  }
  return 0;
}

uint16_t mx_64::read_large_register(mx_64::register_byte p_register)
{
  using namespace std::chrono_literals;

  std::array<hal::byte, 8> send_bytes = { 0xFF, 0xFF, m_id,
                                          0x04, 0x02, (hal::byte)p_register,
                                          0x02, 0x00 };
  hal::byte const temp = std::accumulate(&send_bytes[2], &send_bytes[7], 0);
  send_bytes[7] = ~temp;
  hal::write(*m_serial, send_bytes, hal::never_timeout());

  try {
    auto response =
      hal::read<8>(*m_serial, hal::create_timeout(*m_clock, 500ms));
    if (response[0] == 0xFF && response[1] == 0xFF) {
      // device responded
      // TODO check full packet and checksum
      return (response[5] | (response[6] << 8));
    }
  } catch (hal::timed_out const&) {
    return 0;
  }
  return 0;
}

}  // namespace hal::actuator
