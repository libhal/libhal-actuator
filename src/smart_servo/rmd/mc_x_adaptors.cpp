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

#include <libhal-actuator/smart_servo/rmd/mc_x.hpp>

namespace hal::actuator {
rmd_mc_x_servo::rmd_mc_x_servo(rmd_mc_x& p_mc_x, hal::rpm p_max_speed)
  : m_mc_x(&p_mc_x)
  , m_max_speed(p_max_speed)
{
}

void rmd_mc_x_servo::driver_position(hal::degrees p_position)
{
  m_mc_x->position_control(p_position, m_max_speed);
}

rmd_mc_x_motor::rmd_mc_x_motor(rmd_mc_x& p_mc_x, hal::rpm p_max_speed)
  : m_mc_x(&p_mc_x)
  , m_max_speed(p_max_speed)
{
}

void rmd_mc_x_motor::driver_power(float p_power)
{
  m_mc_x->velocity_control(m_max_speed * p_power);
}

rmd_mc_x_temperature::rmd_mc_x_temperature(rmd_mc_x& p_mc_x)
  : m_mc_x(&p_mc_x)
{
}

hal::celsius rmd_mc_x_temperature::driver_read()
{
  m_mc_x->feedback_request(hal::actuator::rmd_mc_x::read::multi_turns_angle);
  return m_mc_x->feedback().temperature();
}

rmd_mc_x_rotation::rmd_mc_x_rotation(rmd_mc_x& p_mc_x)
  : m_mc_x(&p_mc_x)
{
}

hal::rotation_sensor::read_t rmd_mc_x_rotation::driver_read()
{
  m_mc_x->feedback_request(hal::actuator::rmd_mc_x::read::status_2);
  return { .angle = m_mc_x->feedback().angle() };
}

rmd_mc_x_current_sensor::rmd_mc_x_current_sensor(rmd_mc_x& p_mc_x)
  : m_mc_x(&p_mc_x)
{
}

hal::ampere rmd_mc_x_current_sensor::driver_read()
{
  m_mc_x->feedback_request(hal::actuator::rmd_mc_x::read::status_2);

  return m_mc_x->feedback().current();
}

rmd_mc_x_motor make_motor(rmd_mc_x& p_mc_x, hal::rpm p_max_speed)
{
  return { p_mc_x, p_max_speed };
}
rmd_mc_x_rotation make_rotation_sensor(rmd_mc_x& p_mc_x)
{
  return { p_mc_x };
}
rmd_mc_x_servo make_servo(rmd_mc_x& p_mc_x, hal::rpm p_max_speed)
{
  return { p_mc_x, p_max_speed };
}
rmd_mc_x_temperature make_temperature_sensor(rmd_mc_x& p_mc_x)
{
  return { p_mc_x };
}

rmd_mc_x_current_sensor make_current_sensor(rmd_mc_x& p_mc_x)
{
  return { p_mc_x };
}
}  // namespace hal::actuator
