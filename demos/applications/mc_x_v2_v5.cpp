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

#include <exception>

#include <libhal-actuator/smart_servo/rmd/mc_x_v2.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/can.hpp>
#include <libhal/pointers.hpp>
#include <libhal/servo.hpp>
#include <libhal/units.hpp>
#include <resource_list.hpp>

void application()
{
  using namespace std::literals;
  using namespace hal::literals;

  auto clock = resources::clock();
  auto manager = resources::can_bus_manager();
  auto transceiver = resources::can_transceiver();
  auto filter = resources::can_identifier_filter();
  auto console = resources::console();
  // Needs to be set to this baud rate to work with the default firmware CAN
  // baud rate.
  // manager->baud_rate(1_MHz);
  manager->filter_mode(hal::can_bus_manager::accept::all);
  manager->bus_on();

  hal::print(*console, "RMD MC-X Smart Servo Application Starting...\n\n"sv);

  constexpr std::uint16_t starting_device_address = 0x140;
  std::uint16_t address_offset = 0;

  while (true) {
    try {
      auto const address = starting_device_address + address_offset;
      hal::print<32>(*console, "Using address: 0x%04X\n", address);
      hal::actuator::rmd_mc_x_v2 mc_x(
        *transceiver, *filter, *clock, 36.0f, address);

      auto motor = mc_x.acquire_velocity_motor();
      auto servo = mc_x.acquire_velocity_servo();
      auto temperature_sensor = mc_x.acquire_temperature_sensor();
      auto rotation_sensor = mc_x.acquire_rotation_sensor();

      hal::v5::basic_servo* basic_servo = &servo;

      auto print_feedback = [&](unsigned p_seconds = 8U) {
        for (unsigned i = 0; i < p_seconds * 2; i++) {
          hal::print<128>(*console,
                          ">>> temperature = %f C, shaft angle = %f deg\n",
                          temperature_sensor.read(),
                          rotation_sensor.read().angle);
          hal::delay(*clock, 500ms);
        }
      };

      hal::delay(*clock, 500ms);

      while (true) {
        hal::delay(*clock, 1000ms);
        hal::print(*console, "servo velocity configured to 20rpm\n"sv);
        servo.configure({ .velocity = 20.0_rpm });
        basic_servo->position(0.0_deg);
        hal::print(*console, "servo position 0deg\n"sv);
        print_feedback();

        basic_servo->position(-45.0_deg);
        hal::print(*console, "servo position -45deg\n"sv);
        print_feedback();

        basic_servo->position(90.0_deg);
        hal::print(*console, "servo position 90deg\n"sv);
        print_feedback();

        basic_servo->position(180.0_deg);
        hal::print(*console, "servo position 180deg\n"sv);
        print_feedback();

        hal::print(*console, "servo position 0deg\n"sv);
        basic_servo->position(0.0_deg);
        print_feedback();

        hal::print(*console, "motor drive 20rpm\n"sv);
        motor.drive(10.0_rpm);
        print_feedback();

        hal::print(*console, "motor drive -20rpm\n"sv);
        motor.drive(-10.0_rpm);
        print_feedback();
      }
    } catch (hal::timed_out const&) {
      hal::print(
        *console,
        "hal::timed_out exception! which means that the device did not "
        "respond. Moving to the next device address in the list.\n"sv);
    } catch (hal::resource_unavailable_try_again const& p_error) {
      hal::print(*console, "hal::resource_unavailable_try_again\n"sv);
      if (p_error.instance() == &transceiver) {
        hal::print(
          *console,
          "\n"
          "The CAN peripheral has received no acknowledgements from any other "
          "device on the bus. It appears as if the peripheral is not connected "
          "to a can network. This can happen if the baud rate is incorrect, "
          "the CAN transceiver is not functioning, or the devices on the bus "
          "are not responding."
          "\n"
          "Calling terminate!"
          "\n"
          "Consider powering down the system and checking all of your "
          "connections before restarting the application."sv);
        std::terminate();
      }
      // otherwise keep trying with other addresses
    } catch (...) {
      hal::print(*console, "Unknown exception caught in (...) block\n"sv);
      throw;  // see if anyone else can handle the exception
    }

    address_offset = (address_offset + 1) % 32;
    hal::delay(*clock, 200ms);
  }
}
