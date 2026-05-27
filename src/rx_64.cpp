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

#include <cstdint>

#include <algorithm>
#include <bitset>
#include <numeric>

#include <libhal-actuator/rx_64.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

namespace hal::actuator {

constexpr auto position_raw_range = std::make_pair<u16, u16>(0, 1023);
constexpr auto max_degree_range =
  std::make_pair<hal::degrees, hal::degrees>(0.0f, 300.0f);

rx_64::rx_64(hal::strong_ptr<hal::serial> const& p_serial,
             config const& p_settings,
             hal::strong_ptr<hal::steady_clock> const& p_clock)
  : m_serial(p_serial)
  , m_clock(p_clock)
  , m_id(p_settings.id)
{
  using namespace std::chrono_literals;
  m_serial->configure({ .baud_rate = p_settings.baud_rate });
  baud_rate(p_settings.baud_rate);
  hal::delay(*m_clock, 5ms);
  torque_enable(true);
  hal::delay(*m_clock, 5ms);
  min_angle(p_settings.min_angle);
  m_range.first = p_settings.min_angle;
  hal::delay(*m_clock, 5ms);
  max_angle(p_settings.max_angle);
  m_range.second = p_settings.max_angle;
  hal::delay(*m_clock, 5ms);
};

// TODO: consider keeping this function
bool rx_64::ping_id(uint8_t p_id)
{
  using namespace std::chrono_literals;
  std::array<hal::byte, 6> send_bytes = { 0xFF, 0xFF, (hal::byte)p_id,
                                          0x02, 0x01, 0x00 };
  hal::byte const checksum = std::accumulate(&send_bytes[2], &send_bytes[5], 0);
  send_bytes[5] = ~checksum;
  hal::write(*m_serial, send_bytes, hal::never_timeout());

  try {
    auto response =
      hal::read<6>(*m_serial, hal::create_timeout(*m_clock, 500ms));
    if (response[0] == 0xFF && response[1] == 0xFF) {
      // device responded, id is valid
      m_id = p_id;
    }
  } catch (hal::timed_out const&) {
    return false;
  }
  return true;
}

void rx_64::led(bool p_on)
{
  hal::byte toggle_byte = 0x00;
  if (p_on) {
    toggle_byte = 0x01;
  }
  write_register(register_byte::led_toggle, std::array{ toggle_byte });
}

bool rx_64::is_moving()
{
  auto const response = rx_64::read_register<1>(register_byte::moving_status);
  if (response[0] == 0x01) {
    return true;
  }
  return false;
}

float rx_64::speed()
{
  auto const bytes = rx_64::read_register<2>(register_byte::present_speed);
  uint16_t const response = (bytes[0] | (bytes[1] << 8));
  std::bitset<16> bits{ response };
  bool const clockwise = bits[9];  // 10th bit is direction
  bits.set(9, false);
  float const rpms = static_cast<float>(response) * 0.111f;
  if (clockwise) {
    return rpms;
  }
  return -rpms;
}

float rx_64::voltage()
{
  auto const response =
    rx_64::read_register<1>(register_byte::present_voltage)[0];
  return (static_cast<float>(response) / 10);
}

uint8_t rx_64::temperature()
{
  return rx_64::read_register<1>(register_byte::present_temp)[0];
}

float rx_64::torque_limit()
{
  auto const bytes = rx_64::read_register<2>(register_byte::torque_limit);
  uint16_t const response = (bytes[0] | (bytes[1] << 8));
  return (static_cast<float>(response) / 1023) * 100.0f;
}

bool rx_64::torque_enable()
{
  auto const enabled_byte =
    rx_64::read_register<1>(rx_64::register_byte::torque_enable)[0];
  return enabled_byte == 0x01;
}

uint8_t rx_64::temperature_limit()
{
  return rx_64::read_register<1>(register_byte::temp_limit)[0];
}

float rx_64::min_voltage()
{
  auto const response = rx_64::read_register<1>(register_byte::min_voltage)[0];
  return (static_cast<float>(response) / 10);
}

float rx_64::max_voltage()
{
  auto const response = rx_64::read_register<1>(register_byte::max_voltage)[0];
  return (static_cast<float>(response) / 10);
}

hertz rx_64::baud_rate()
{
  auto const response = rx_64::read_register<1>(register_byte::baud_rate)[0];
  switch (response) {
    case 1:
      return 1000000;
    case 3:
      return 500000;
    case 4:
      return 400000;
    case 7:
      return 250000;
    case 9:
      return 200000;
    case 16:
      return 115200;
    case 103:
      return 19200;
    case 207:
      return 9600;
    case 34:
    default:
      return 57600;
  }
}

uint16_t rx_64::return_delay_time()
{
  auto const response = rx_64::read_register<1>(register_byte::return_delay)[0];
  return (response * 2);
}

uint8_t rx_64::id()
{
  return m_id;
}

hal::degrees rx_64::min_angle()
{
  auto const bytes = rx_64::read_register<2>(register_byte::cw_limit);
  uint16_t const angle_byte = (bytes[0] | (bytes[1] << 8));
  return hal::map(angle_byte, position_raw_range, max_degree_range);
}

hal::degrees rx_64::max_angle()
{
  auto const bytes = rx_64::read_register<2>(register_byte::ccw_limit);
  uint16_t const angle_byte = (bytes[0] | (bytes[1] << 8));
  return hal::map(angle_byte, position_raw_range, max_degree_range);
}

hal::degrees rx_64::position()
{
  auto const bytes = rx_64::read_register<2>(register_byte::present_position);
  uint16_t const angle_byte = (bytes[0] | (bytes[1] << 8));
  return hal::map(angle_byte, position_raw_range, max_degree_range);
}

uint16_t rx_64::punch()
{
  auto const bytes = rx_64::read_register<2>(register_byte::punch);
  return (bytes[0] | (bytes[1] << 8));
}

float rx_64::moving_speed()
{
  auto const bytes = rx_64::read_register<2>(register_byte::moving_speed);
  uint16_t const response = (bytes[0] | (bytes[1] << 8));
  return (static_cast<float>(response) / 8.9737f);
}

void rx_64::position(hal::degrees p_angle)
{
  auto const clamped_angle = std::clamp(p_angle, m_range.first, m_range.second);
  auto const angle_byte = static_cast<uint16_t>(
    hal::map(clamped_angle, max_degree_range, position_raw_range));
  hal::byte const value_low = angle_byte;
  hal::byte const value_hi = (angle_byte >> 8);
  write_register(register_byte::goal_position,
                 std::array{ value_low, value_hi });
}

void rx_64::torque_enable(bool p_enable)
{
  hal::byte data_byte = 0x00;
  if (p_enable) {
    data_byte = 0x01;
  }
  write_register(register_byte::torque_enable, std::array{ data_byte });
}

void rx_64::torque_limit(float p_percent)
{
  auto const clamped_percent = std::clamp(p_percent, 0.0f, 100.0f);
  auto const value = static_cast<uint16_t>(hal::map(
    clamped_percent, std::make_pair(0.0f, 100.0f), std::make_pair(0, 1023)));
  // auto const value = static_cast<uint16_t>(1023 * (clamped_percent / 100));
  hal::byte const value_low = value;
  hal::byte const value_hi = (value >> 8);
  write_register(register_byte::torque_limit,
                 std::array{ value_low, value_hi });
}

void rx_64::temperature_limit(uint8_t p_temperature)
{
  auto const clamped_temp = std::clamp(p_temperature, (uint8_t)0, (uint8_t)100);
  write_register(register_byte::temp_limit, std::array{ clamped_temp });
}

void rx_64::min_voltage(float p_voltage)
{
  auto const clamped_volt = std::clamp(p_voltage, 5.0f, 25.0f);
  auto const value = static_cast<uint8_t>(clamped_volt * 10);
  write_register(register_byte::min_voltage, std::array{ value });
}

void rx_64::max_voltage(float p_voltage)
{
  auto const clamped_volt = std::clamp(p_voltage, 5.0f, 25.0f);
  auto const value = static_cast<uint8_t>(clamped_volt * 10);
  write_register(register_byte::max_voltage, std::array{ value });
}

void rx_64::baud_rate(hertz p_baud)
{
  uint8_t baud_byte = 0x00;
  switch ((uint32_t)p_baud) {
    case 1000000:
      baud_byte = 1;
      break;
    case 500000:
      baud_byte = 3;
      break;
    case 400000:
      baud_byte = 4;
      break;
    case 250000:
      baud_byte = 7;
      break;
    case 200000:
      baud_byte = 9;
      break;
    case 115200:
      baud_byte = 16;
      break;
    case 19200:
      baud_byte = 103;
      break;
    case 9600:
      baud_byte = 207;
      break;
    case 57600:
    default:
      baud_byte = 34;
      break;
  }
  write_register(register_byte::baud_rate, std::array{ baud_byte });
}

void rx_64::return_delay_time(uint16_t p_microseconds)
{
  uint8_t const value = std::clamp((p_microseconds / 2), 0, 254);
  write_register(register_byte::return_delay, std::array{ value });
}

void rx_64::id(uint8_t p_id)
{
  m_id = std::clamp(p_id, (uint8_t)0, (uint8_t)253);
  write_register(register_byte::id, std::array{ p_id });
}

void rx_64::min_angle(hal::degrees p_angle)
{
  m_range.first =
    std::clamp(p_angle, max_degree_range.first, max_degree_range.second);
  auto const angle_byte = static_cast<uint16_t>(
    hal::map(m_range.first, max_degree_range, position_raw_range));
  hal::byte const value_low = angle_byte;
  hal::byte const value_hi = (angle_byte >> 8);
  write_register(register_byte::cw_limit, std::array{ value_low, value_hi });
}

void rx_64::max_angle(hal::degrees p_angle)
{
  m_range.second =
    std::clamp(p_angle, max_degree_range.first, max_degree_range.second);
  auto const angle_byte = static_cast<uint16_t>(
    hal::map(m_range.second, max_degree_range, position_raw_range));
  hal::byte const value_low = angle_byte;
  hal::byte const value_hi = (angle_byte >> 8);
  write_register(register_byte::ccw_limit, std::array{ value_low, value_hi });
}

void rx_64::speed(float p_rpms)
{
  auto const clamped_rpm = std::clamp(p_rpms, 0.0f, 114.0f);
  auto const speed_byte = static_cast<u16>(clamped_rpm * 8.9737);
  hal::byte const value_low = speed_byte;
  hal::byte const value_hi = (speed_byte >> 8);
  write_register(register_byte::moving_speed,
                 std::array{ value_low, value_hi });
}

void rx_64::sync_position(hal::degrees p_angle, rx_64 p_opposing_servo)
{
  auto const clamped_angle = std::clamp(p_angle, m_range.first, m_range.second);
  auto const angle_byte = static_cast<u16>(
    hal::map(clamped_angle, max_degree_range, position_raw_range));
  auto const reversed_angle = position_raw_range.second - angle_byte;

  uint8_t const angle_low = angle_byte;
  uint8_t const angle_hi = (angle_byte >> 8);
  uint8_t const reversed_low = reversed_angle;
  uint8_t const reversed_hi = (reversed_angle >> 8);

  auto const servo2_id = p_opposing_servo.id();
  std::array<hal::byte, 14> send_bytes = {
    0xFF, 0xFF,      0xFE,     0x0A,      0x83,         0x1E,        0x02,
    m_id, angle_low, angle_hi, servo2_id, reversed_low, reversed_hi, 0x00
  };
  hal::byte const checksum =
    std::accumulate(&send_bytes[2], &send_bytes[13], 0);
  send_bytes[13] = ~checksum;
  hal::write(*m_serial, send_bytes, hal::never_timeout());
}
}  // namespace hal::actuator
