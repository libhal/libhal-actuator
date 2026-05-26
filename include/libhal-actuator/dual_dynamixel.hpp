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

#include <libhal-actuator/mx_64.hpp>
#include <libhal-actuator/rx_64.hpp>

namespace hal::actuator {
class dual_dynamixel
{
public:
  dual_dynamixel(hal::strong_ptr<rx_64> p_lead_servo,
                 hal::strong_ptr<rx_64> p_opposing_servo);

  void position(hal::degrees p_angle);
  void set_torque_limit(float p_percent);
  void set_speed(float p_rpms);

private:
  hal::strong_ptr<rx_64> m_lead_servo;
  hal::strong_ptr<rx_64> m_opposing_servo;
  rx_64::angle_range m_range;
};
}  // namespace hal::actuator
