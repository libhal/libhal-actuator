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

#include <cmath>

#include <libhal-actuator/smart_servo/rmd/drc.hpp>

namespace hal::actuator {
rmd_drc_servo::rmd_drc_servo(rmd_drc& p_drc, hal::rpm p_max_speed)
  : m_drc(&p_drc)
  , m_max_speed(p_max_speed)
{
}

void rmd_drc_servo::driver_position(hal::degrees p_position)
{
  m_drc->position_control(p_position, m_max_speed);
}

rmd_drc_temperature_sensor::rmd_drc_temperature_sensor(rmd_drc& p_drc)
  : m_drc(&p_drc)
{
}

hal::celsius rmd_drc_temperature_sensor::driver_read()
{
  m_drc->feedback_request(hal::actuator::rmd_drc::read::status_2);
  return m_drc->feedback().temperature();
}

rmd_drc_rotation_sensor::rmd_drc_rotation_sensor(rmd_drc& p_drc)
  : m_drc(&p_drc)
{
}

hal::rotation_sensor::read_t rmd_drc_rotation_sensor::driver_read()
{
  m_drc->feedback_request(hal::actuator::rmd_drc::read::multi_turns_angle);
  return { .angle = m_drc->feedback().angle() };
}

actuator::rmd_drc_rotation_sensor make_rotation_sensor(rmd_drc& p_drc)
{
  return { p_drc };
}

actuator::rmd_drc_servo make_servo(rmd_drc& p_drc, hal::rpm p_max_speed)
{
  return { p_drc, std::abs(p_max_speed) };
}

actuator::rmd_drc_temperature_sensor make_temperature_sensor(rmd_drc& p_drc)
{
  return { p_drc };
}

rmd_drc_motor::rmd_drc_motor(rmd_drc& p_drc, hal::rpm p_max_speed)
  : m_drc(&p_drc)
  , m_max_speed(p_max_speed)
{
}

void rmd_drc_motor::driver_power(float p_power)
{
  m_drc->velocity_control(m_max_speed * p_power);
}

rmd_drc_motor make_motor(rmd_drc& p_drc, hal::rpm p_max_speed)
{
  return { p_drc, std::abs(p_max_speed) };
}

rmd_drc_angular_velocity_sensor::rmd_drc_angular_velocity_sensor(rmd_drc& p_drc)
  : m_drc(&p_drc)
{
}

hal::rpm rmd_drc_angular_velocity_sensor::driver_read()
{
  m_drc->feedback_request(rmd_drc::read::status_2);
  return m_drc->feedback().speed();
}

rmd_drc_angular_velocity_sensor make_angular_velocity_sensor(rmd_drc& p_drc)
{
  return { p_drc };
}
}  // namespace hal::actuator
