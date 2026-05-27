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

#include <array>
#include <cstdint>
#include <libhal-actuator/mx_64.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/units.hpp>

#include <resource_list.hpp>

void application()
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto clock = resources::clock();
  auto console = resources::console();
  auto uart = resources::uart2();

  hal::print(*console, "Dynamixel Demo Starting...\n");

  hal::actuator::mx_64::config mx_servo_config = {
    .baud_rate = 57600, .id = 0, .min_angle = 0, .max_angle = 360
  };

  auto mx_servo = hal::actuator::mx_64(uart, mx_servo_config, clock);
  auto min_angle = mx_servo.get_min_angle();
  hal::print<32>(*console, "\nMin Angle: %.2f", min_angle);
  hal::delay(*clock, 10ms);

  auto max_angle = mx_servo.get_max_angle();
  hal::print<32>(*console, "\nMax Angle: %.2f", max_angle);
  hal::delay(*clock, 10ms);

  auto torque_limit = mx_servo.get_torque_limit();
  hal::print<32>(*console, "\nTorque Limit: %.2f", torque_limit);
  hal::delay(*clock, 10ms);

  auto temp_limit = mx_servo.get_temp_limit();
  hal::print<32>(*console, "\nTemp Limit: %d", temp_limit);
  hal::delay(*clock, 10ms);

  auto min_voltage = mx_servo.get_min_voltage();
  hal::print<32>(*console, "\nMin Volt: %.2f", min_voltage);
  hal::delay(*clock, 10ms);

  auto max_voltage = mx_servo.get_max_voltage();
  hal::print<32>(*console, "\nMax Volt: %.2f", max_voltage);
  hal::delay(*clock, 10ms);

  auto return_delay = mx_servo.get_return_delay_time();
  hal::print<32>(*console, "\nReturn Delay: %d", return_delay);
  hal::delay(*clock, 10ms);

  auto punch = mx_servo.get_punch();
  hal::print<32>(*console, "\nPunch: %d", punch);
  hal::delay(*clock, 10ms);

  mx_servo.set_torque_enable(true);
  hal::delay(*clock, 10ms);

  auto torque_enable = mx_servo.get_torque_enable();
  hal::print<32>(*console, "\nTorque Enable: %d\n", torque_enable);
  hal::delay(*clock, 10ms);

  // loop and move servo
  while (true) {
    hal::print(*console, "90 degrees\n");
    mx_servo.position(90);
    hal::delay(*clock, 3000ms);
    hal::print(*console, "180 degrees\n");
    mx_servo.position(180);
    hal::delay(*clock, 3000ms);
    hal::print(*console, "270 degrees\n");
    mx_servo.position(270);
    hal::delay(*clock, 3000ms);
    hal::print(*console, "180 degrees\n");
    mx_servo.position(180);
    hal::delay(*clock, 3000ms);
  }
}
