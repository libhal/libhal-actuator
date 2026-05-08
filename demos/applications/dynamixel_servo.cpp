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
#include <libhal-actuator/rx_64.hpp>
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

  hal::actuator::rx_64::config servo_config = { .torque_limit = 100,
                                                .temp_limit = 80,
                                                .min_voltage = 6.0,
                                                .max_voltage = 19.0,
                                                .baud_rate = 34,
                                                .return_delay_time = 250,
                                                .id = 4,
                                                .min_angle = 0,
                                                .max_angle = 300 };
  uart->configure({ .baud_rate = 57600 });

  auto servo = hal::actuator::rx_64(uart, servo_config, clock);
  auto min_angle = servo.get_min_angle();
  hal::print<48>(*console, "\nMin Angle %f", min_angle);
  hal::delay(*clock, 10ms);
  auto max_angle = servo.get_max_angle();
  hal::print<48>(*console, "\nMax Angle %f", max_angle);
  hal::delay(*clock, 10ms);

  servo.set_torque_enable(true);
  hal::delay(*clock, 10ms);

  // loop and move servo
  while (true) {
    servo.position(0);
    hal::delay(*clock, 3000ms);
    servo.position(150);
    hal::delay(*clock, 3000ms);
    servo.position(300);
    hal::delay(*clock, 3000ms);
    servo.position(150);
    hal::delay(*clock, 3000ms);
  }
}
