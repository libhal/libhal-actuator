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

  hal::print(*console, "Dynamixel Scan Application Starting...\n");

  hal::actuator::rx_64::config servo_config = { .torque_limit = 100,
                                                .temp_limit = 80,
                                                .min_voltage = 6.0,
                                                .max_voltage = 19.0,
                                                .baud_rate = 34,
                                                .return_delay_time = 250,
                                                .id = 1,
                                                .min_angle = 0,
                                                .max_angle = 300 };

  auto servo = hal::actuator::rx_64(uart, servo_config, clock);

  std::array<hal::hertz, 9> available_bauds = { 9600,   19200,  57600,
                                                115200, 200000, 250000,
                                                400000, 500000, 1000000 };
  bool device_found = false;
  bool baud_rate_found = false;
  // loop through bauds
  for (auto current_baud : available_bauds) {
    uart->configure({ .baud_rate = current_baud });
    // loop through addresses
    for (uint8_t i = 0; i < 254; i++) {
      // ping with address i, result into device_found
      hal::print<32>(*console, "Checking ID: %d\n", i);
      device_found = servo.ping_id(i);
      if (device_found) {
        baud_rate_found = true;
        hal::print<32>(*console,
                       "Servo found with baud rate %f at id: %d\n",
                       current_baud,
                       i);
        servo.set_id(i);
        // turn on led to id which servo has id reported
        servo.led_toggle(true);
        hal::delay(*clock, 3s);
        servo.led_toggle(false);
      }
    }
    if (baud_rate_found) {
      break;
    }
  }
}
