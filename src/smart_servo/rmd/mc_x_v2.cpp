// Copyright 2024 Khalil Estell
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

#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>

#include <cstdint>

#include <libhal-util/can.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/map.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/error.hpp>

#include "common.hpp"
#include "mc_x_constants.hpp"

namespace hal::actuator {
namespace {
inline std::int32_t rpm_to_mc_x_speed(rpm p_rpm, float p_dps_per_lsb)
{
  static constexpr float dps_per_rpm = (1.0f / 1.0_deg_per_sec);
  float const dps_float = (p_rpm * dps_per_rpm) / p_dps_per_lsb;
  return bounds_check<std::int32_t>(dps_float);
}

inline hal::u32 motor_id(hal::can_message_finder const& p_message_finder)
{
  // Paranoid safety assertions, such that a change to this value in the future
  // alerts the developer to see this function.
  static_assert(response_id_offset == 0x100);
  // Safe so long as the ID is bounds checked at construction to be between
  // 0x140 and 0x160.
  return p_message_finder.id() - response_id_offset;
}
}  // namespace

hal::ampere rmd_mc_x_v2::feedback_t::current() const noexcept
{
  static constexpr auto amps_per_lsb = 0.1_A;
  return static_cast<float>(raw_current) * amps_per_lsb;
}

hal::rpm rmd_mc_x_v2::feedback_t::speed() const noexcept
{
  static constexpr auto velocity_per_lsb = 1.0_deg_per_sec;
  return static_cast<float>(raw_speed) * velocity_per_lsb;
}

hal::volts rmd_mc_x_v2::feedback_t::volts() const noexcept
{
  static constexpr float volts_per_lsb = 0.1f;
  return static_cast<float>(raw_volts) * volts_per_lsb;
}

hal::celsius rmd_mc_x_v2::feedback_t::temperature() const noexcept
{
  static constexpr float celsius_per_lsb = 1.0f;
  return static_cast<float>(raw_motor_temperature) * celsius_per_lsb;
}

hal::degrees rmd_mc_x_v2::feedback_t::angle() const noexcept
{
  return static_cast<float>(raw_multi_turn_angle) * dps_per_lsb_speed;
}

bool rmd_mc_x_v2::feedback_t::motor_stall() const noexcept
{
  return raw_error_state & motor_stall_mask;
}
bool rmd_mc_x_v2::feedback_t::low_pressure() const noexcept
{
  return raw_error_state & low_pressure_mask;
}
bool rmd_mc_x_v2::feedback_t::over_voltage() const noexcept
{
  return raw_error_state & over_voltage_mask;
}
bool rmd_mc_x_v2::feedback_t::over_current() const noexcept
{
  return raw_error_state & over_current_mask;
}
bool rmd_mc_x_v2::feedback_t::power_overrun() const noexcept
{
  return raw_error_state & power_overrun_mask;
}
bool rmd_mc_x_v2::feedback_t::speeding() const noexcept
{
  return raw_error_state & speeding_mask;
}
bool rmd_mc_x_v2::feedback_t::over_temperature() const noexcept
{
  return raw_error_state & over_temperature_mask;
}
bool rmd_mc_x_v2::feedback_t::encoder_calibration_error() const noexcept
{
  return raw_error_state & encoder_calibration_error_mask;
}

rmd_mc_x_v2::rmd_mc_x_v2(hal::can_transceiver& p_can_transceiver,
                         hal::steady_clock& p_clock,
                         float p_gear_ratio,  // NOLINT
                         hal::u32 p_device_id,
                         hal::time_duration p_max_response_time)
  : m_feedback{}
  , m_clock(&p_clock)
  , m_can(p_can_transceiver, p_device_id + response_id_offset)
  , m_gear_ratio(p_gear_ratio)
  , m_device_id(p_device_id)
  , m_max_response_time(p_max_response_time)
{
  bool const valid_device_id =
    first_device_address <= p_device_id && p_device_id <= last_device_address;

  bool const invalid_baud_rate = p_can_transceiver.baud_rate() != 1_MHz;

  if (not valid_device_id || invalid_baud_rate) {
    hal::safe_throw(hal::argument_out_of_domain(this));
  }

  // Will throw hal::timed_out if the device is not present on the bus to
  // respond to this.
  feedback_request(read::status_1_and_error_flags);
}

void rmd_mc_x_v2::send(std::array<hal::byte, 8> p_payload)
{
  hal::can_message const payload{
    .id = motor_id(m_can),
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

void rmd_mc_x_v2::velocity_control(rpm p_rpm)
{
  auto const speed_data = rpm_to_mc_x_speed(p_rpm, dps_per_lsb_speed);

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

void rmd_mc_x_v2::position_control(degrees p_angle, rpm p_rpm)  // NOLINT
{
  static constexpr float deg_per_lsb = 0.01f;
  auto const angle = p_angle / deg_per_lsb;
  auto const angle_data = bounds_check<std::int32_t>(angle);
  auto const speed_data =
    rpm_to_mc_x_speed(std::abs(p_rpm * m_gear_ratio), dps_per_lsb_angle);

  send({
    hal::value(actuate::position),
    0x00,
    static_cast<hal::byte>((speed_data >> 0) & 0xFF),
    static_cast<hal::byte>((speed_data >> 8) & 0xFF),
    static_cast<hal::byte>((angle_data >> 0) & 0xFF),
    static_cast<hal::byte>((angle_data >> 8) & 0xFF),
    static_cast<hal::byte>((angle_data >> 16) & 0xFF),
    static_cast<hal::byte>((angle_data >> 24) & 0xFF),
  });
}

void rmd_mc_x_v2::feedback_request(read p_command)
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

void rmd_mc_x_v2::system_control(system p_system_command)
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

rmd_mc_x_v2::feedback_t const& rmd_mc_x_v2::feedback() const
{
  return m_feedback;
}

void rmd_mc_x_v2::handle_message(can_message const& p_message)
{
  m_feedback.message_number++;

  if (p_message.length != 8 ||
      p_message.id != m_device_id + response_id_offset) {
    return;
  }

  switch (p_message.payload[0]) {
    case hal::value(read::status_2):
    case hal::value(actuate::torque):
    case hal::value(actuate::speed):
    case hal::value(actuate::position): {
      auto& data = p_message.payload;
      m_feedback.raw_motor_temperature = static_cast<int8_t>(data[1]);
      m_feedback.raw_current = static_cast<int16_t>((data[3] << 8) | data[2]);
      m_feedback.raw_speed = static_cast<int16_t>((data[5] << 8) | data[4]);
      m_feedback.encoder = static_cast<int16_t>((data[7] << 8) | data[6]);
      break;
    }
    case hal::value(read::status_1_and_error_flags): {
      auto& data = p_message.payload;
      m_feedback.raw_motor_temperature = static_cast<int8_t>(data[1]);
      // data[3] = Brake release command
      m_feedback.raw_volts = static_cast<int16_t>((data[5] << 8) | data[4]);
      auto error_state = data[7] << 8 | data[6];
      m_feedback.raw_error_state = static_cast<int16_t>(error_state);
      break;
    }
    case hal::value(read::multi_turns_angle): {
      auto& data = p_message.payload;
      m_feedback.raw_multi_turn_angle = static_cast<std::int32_t>(
        data[7] << 24 | data[6] << 16 | data[5] << 8 | data[4]);
      break;
    }
    default:
      return;
  }
}

// =============================================================================
//
// Interface Implementation constructors
//
// =============================================================================

rmd_mc_x_v2::servo::servo(rmd_mc_x_v2& p_mc_x, hal::rpm p_max_speed)
  : m_mc_x(&p_mc_x)
  , m_max_speed(p_max_speed)
{
}
rmd_mc_x_v2::motor::motor(rmd_mc_x_v2& p_mc_x, hal::rpm p_max_speed)
  : m_mc_x(&p_mc_x)
  , m_max_speed(p_max_speed)
{
}
rmd_mc_x_v2::temperature::temperature(rmd_mc_x_v2& p_mc_x)
  : m_mc_x(&p_mc_x)
{
}
rmd_mc_x_v2::rotation::rotation(rmd_mc_x_v2& p_mc_x)
  : m_mc_x(&p_mc_x)
{
}
rmd_mc_x_v2::current_sensor::current_sensor(rmd_mc_x_v2& p_mc_x)
  : m_mc_x(&p_mc_x)
{
}

// =============================================================================
//
// Interface Implementations
//
// =============================================================================

void rmd_mc_x_v2::servo::driver_position(hal::degrees p_position)
{
  m_mc_x->position_control(p_position, m_max_speed);
}

void rmd_mc_x_v2::motor::driver_power(float p_power)
{
  m_mc_x->velocity_control(m_max_speed * p_power);
}

hal::celsius rmd_mc_x_v2::temperature::driver_read()
{
  m_mc_x->feedback_request(read::multi_turns_angle);
  return m_mc_x->feedback().temperature();
}

hal::rotation_sensor::read_t rmd_mc_x_v2::rotation::driver_read()
{
  m_mc_x->feedback_request(read::status_2);
  return { .angle = m_mc_x->feedback().angle() };
}

hal::ampere rmd_mc_x_v2::current_sensor::driver_read()
{
  m_mc_x->feedback_request(read::status_2);

  return m_mc_x->feedback().current();
}

// =============================================================================
//
// Object acquisition functions
//
// =============================================================================

rmd_mc_x_v2::motor rmd_mc_x_v2::acquire_motor(hal::rpm p_max_speed)
{
  return { *this, p_max_speed };
}
rmd_mc_x_v2::rotation rmd_mc_x_v2::acquire_rotation_sensor()
{
  return { *this };
}
rmd_mc_x_v2::servo rmd_mc_x_v2::acquire_servo(hal::rpm p_max_speed)
{
  return { *this, p_max_speed };
}
rmd_mc_x_v2::temperature rmd_mc_x_v2::acquire_temperature_sensor()
{
  return { *this };
}

rmd_mc_x_v2::current_sensor rmd_mc_x_v2::acquire_current_sensor()
{
  return { *this };
}
}  // namespace hal::actuator
