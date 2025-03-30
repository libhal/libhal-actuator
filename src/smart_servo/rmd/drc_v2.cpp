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

#include <libhal-actuator/smart_servo/rmd/drc_v2.hpp>

#include <cstdint>

#include <libhal-util/bit.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>
#include <libhal/servo.hpp>

#include "common.hpp"
#include "drc_constants.hpp"

namespace hal::actuator {
namespace {
std::int32_t rpm_to_drc_speed(rpm p_rpm,
                              float p_gear_ratio,
                              float p_dps_per_lsb)
{
  static constexpr float dps_per_rpm = (1.0f / 1.0_deg_per_sec);

  float const dps_float = (p_rpm * p_gear_ratio * dps_per_rpm) / p_dps_per_lsb;

  return bounds_check<std::int32_t>(dps_float);
}
}  // namespace

hal::ampere rmd_drc_v2::feedback_t::current() const noexcept
{
  static constexpr float raw_current_range = 2048.0f;
  static constexpr auto current_range = 33.0_A;
  return hal::map(static_cast<float>(raw_current),
                  std::make_pair(-raw_current_range, raw_current_range),
                  std::make_pair(-current_range, current_range));
}

hal::rpm rmd_drc_v2::feedback_t::speed() const noexcept
{
  static constexpr auto velocity_per_lsb = 1.0_deg_per_sec;
  return static_cast<float>(raw_speed) * velocity_per_lsb;
}

hal::volts rmd_drc_v2::feedback_t::volts() const noexcept
{
  static constexpr float volts_per_lsb = 0.1f;
  return static_cast<float>(raw_volts) * volts_per_lsb;
}

hal::celsius rmd_drc_v2::feedback_t::temperature() const noexcept
{
  static constexpr float celsius_per_lsb = 1.0f;
  return static_cast<float>(raw_motor_temperature) * celsius_per_lsb;
}

hal::degrees rmd_drc_v2::feedback_t::angle() const noexcept
{
  return static_cast<float>(raw_multi_turn_angle) * dps_per_lsb_speed;
}

bool rmd_drc_v2::feedback_t::over_voltage_protection_tripped() const noexcept
{
  return raw_error_state & over_temperature_protection_tripped_mask;
}

bool rmd_drc_v2::feedback_t::over_temperature_protection_tripped()
  const noexcept
{
  return raw_error_state & over_temperature_protection_tripped_mask;
}

rmd_drc_v2::feedback_t const& rmd_drc_v2::feedback() const
{
  return m_feedback;
}

rmd_drc_v2::rmd_drc_v2(hal::can_transceiver& p_can,
                       hal::can_identifier_filter& p_filter,
                       hal::steady_clock& p_clock,
                       float p_gear_ratio,  // NOLINT
                       hal::u32 p_device_id,
                       hal::time_duration p_max_response_time)
  : m_feedback{}
  , m_can(p_can, p_device_id)
  , m_clock(&p_clock)
  , m_gear_ratio(p_gear_ratio)
  , m_max_response_time(p_max_response_time)
{
  p_filter.allow(p_device_id);
  rmd_drc_v2::system_control(system::off);
  rmd_drc_v2::system_control(system::running);
}

void rmd_drc_v2::send(std::array<hal::byte, 8> p_payload)
{
  hal::can_message const payload{
    .id = m_can.id(),
    .length = 8,
    .payload = p_payload,
  };

  // Send payload
  m_can.transceiver().send(payload);

  // Wait for the message number to increment
  auto const deadline = hal::future_deadline(*m_clock, m_max_response_time);
  while (deadline > m_clock->uptime()) {
    if (auto const message = m_can.find(); message.has_value()) {
      handle_message(*message);
      return;
    }
  }

  hal::safe_throw(hal::timed_out(this));
}

void rmd_drc_v2::velocity_control(rpm p_rpm)
{
  auto const speed_data =
    rpm_to_drc_speed(p_rpm, m_gear_ratio, dps_per_lsb_speed);

  send({
    hal::value(actuate::speed),
    0x00,
    0x00,
    0x00,
    static_cast<hal::byte>((speed_data >> 0) & 0xFF),
    static_cast<hal::byte>((speed_data >> 8) & 0xFF),
    static_cast<hal::byte>((speed_data >> 16) & 0xFF),
    static_cast<hal::byte>((speed_data >> 24) & 0xFF),
  });
}

void rmd_drc_v2::position_control(degrees p_angle, rpm p_rpm)  // NOLINT
{
  static constexpr float deg_per_lsb = 0.01f;
  auto const angle = (p_angle * m_gear_ratio) / deg_per_lsb;
  auto const angle_data = bounds_check<std::int32_t>(angle);
  auto const speed_data =
    rpm_to_drc_speed(p_rpm, m_gear_ratio, dps_per_lsb_angle);

  send({
    hal::value(actuate::position_2),
    0x00,
    static_cast<hal::byte>((speed_data >> 0) & 0xFF),
    static_cast<hal::byte>((speed_data >> 8) & 0xFF),
    static_cast<hal::byte>((angle_data >> 0) & 0xFF),
    static_cast<hal::byte>((angle_data >> 8) & 0xFF),
    static_cast<hal::byte>((angle_data >> 16) & 0xFF),
    static_cast<hal::byte>((angle_data >> 24) & 0xFF),
  });
}

void rmd_drc_v2::feedback_request(read p_command)
{
  send({
    hal::value(p_command),
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
  });
}

void rmd_drc_v2::system_control(system p_system_command)
{
  send({
    hal::value(p_system_command),
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
  });
}

void rmd_drc_v2::handle_message(can_message const& p_message)
{
  m_feedback.message_number++;

  switch (p_message.payload[0]) {
    case hal::value(read::status_2):
    case hal::value(actuate::speed):
    case hal::value(actuate::position_2): {
      auto& data = p_message.payload;
      m_feedback.raw_motor_temperature = static_cast<std::int8_t>(data[1]);
      m_feedback.raw_current =
        static_cast<std::int16_t>((data[3] << 8) | data[2] << 0);
      m_feedback.raw_speed =
        static_cast<std::int16_t>((data[5] << 8) | data[4] << 0);
      m_feedback.encoder =
        static_cast<std::int16_t>((data[7] << 8) | data[6] << 0);
      break;
    }
    case hal::value(read::status_1_and_error_flags): {
      auto& data = p_message.payload;
      m_feedback.raw_motor_temperature = static_cast<int8_t>(data[1]);
      m_feedback.raw_volts =
        static_cast<std::int16_t>((data[4] << 8) | data[3]);
      m_feedback.raw_error_state = data[7];
      break;
    }
    case hal::value(read::multi_turns_angle): {
      auto& data = p_message.payload;

      m_feedback.raw_multi_turn_angle = hal::bit_value(0U)
                                          .insert<byte_m<0>>(data[1])
                                          .insert<byte_m<1>>(data[2])
                                          .insert<byte_m<2>>(data[3])
                                          .insert<byte_m<3>>(data[4])
                                          .insert<byte_m<4>>(data[5])
                                          .insert<byte_m<5>>(data[6])
                                          .insert<byte_m<6>>(data[7])
                                          .to<std::int64_t>();
      break;
    }
    default:
      break;
  }
}

// =============================================================================
//
// Interface Implementation constructors
//
// =============================================================================

rmd_drc_v2::servo::servo(rmd_drc_v2& p_drc, hal::rpm p_max_speed)
  : m_drc(&p_drc)
  , m_max_speed(p_max_speed)
{
}

rmd_drc_v2::temperature_sensor::temperature_sensor(rmd_drc_v2& p_drc)
  : m_drc(&p_drc)
{
}

rmd_drc_v2::rotation_sensor::rotation_sensor(rmd_drc_v2& p_drc)
  : m_drc(&p_drc)
{
}

rmd_drc_v2::motor::motor(rmd_drc_v2& p_drc, hal::rpm p_max_speed)
  : m_drc(&p_drc)
  , m_max_speed(p_max_speed)
{
}

rmd_drc_v2::angular_velocity_sensor::angular_velocity_sensor(rmd_drc_v2& p_drc)
  : m_drc(&p_drc)
{
}

// =============================================================================
//
// Object acquisition functions
//
// =============================================================================

rmd_drc_v2::motor rmd_drc_v2::acquire_motor(hal::rpm p_max_speed)
{
  return { *this, std::abs(p_max_speed) };
}

rmd_drc_v2::rotation_sensor rmd_drc_v2::acquire_rotation_sensor()
{
  return { *this };
}

rmd_drc_v2::servo rmd_drc_v2::acquire_servo(hal::rpm p_max_speed)
{
  return { *this, std::abs(p_max_speed) };
}

rmd_drc_v2::temperature_sensor rmd_drc_v2::acquire_temperature_sensor()
{
  return { *this };
}

rmd_drc_v2::angular_velocity_sensor
rmd_drc_v2::acquire_angular_velocity_sensor()
{
  return { *this };
}

// =============================================================================
//
// Interface Implementations
//
// =============================================================================

void rmd_drc_v2::servo::driver_position(hal::degrees p_position)
{
  m_drc->position_control(p_position, m_max_speed);
}

hal::celsius rmd_drc_v2::temperature_sensor::driver_read()
{
  m_drc->feedback_request(read::status_2);
  return m_drc->feedback().temperature();
}

hal::rotation_sensor::read_t rmd_drc_v2::rotation_sensor::driver_read()
{
  m_drc->feedback_request(read::multi_turns_angle);
  return { .angle = m_drc->feedback().angle() };
}

void rmd_drc_v2::motor::driver_power(float p_power)
{
  m_drc->velocity_control(m_max_speed * p_power);
}

hal::rpm rmd_drc_v2::angular_velocity_sensor::driver_read()
{
  m_drc->feedback_request(read::status_2);
  return m_drc->feedback().speed();
}
}  // namespace hal::actuator
