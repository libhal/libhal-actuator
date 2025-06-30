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

namespace hal {
/**
 * @ingroup Serial
 * @brief Write formatted string data to serial buffer and drop return value
 *
 * Uses snprintf internally and writes to a local statically allocated an array.
 * This function will never dynamically allocate like how standard std::printf
 * does.
 *
 * This function does NOT include the NULL character when transmitting the data
 * over the serial port.
 *
 * @tparam buffer_size - Size of the buffer to allocate on the stack to store
 * the formatted message.
 * @tparam Parameters - printf arguments
 * @param p_serial - serial port to write data to
 * @param p_format - printf style null terminated format string
 * @param p_parameters - printf arguments
 */
template<size_t buffer_size, typename... Parameters>
void print(v5::serial& p_serial,
           char const* p_format,
           Parameters... p_parameters)
{
  static_assert(buffer_size > 2);
  constexpr int unterminated_max_string_size =
    static_cast<int>(buffer_size) - 1;

  std::array<char, buffer_size> buffer{};
  int length =
    std::snprintf(buffer.data(), buffer.size(), p_format, p_parameters...);

  if (length > unterminated_max_string_size) {
    // Print out what was able to be written to the buffer
    length = unterminated_max_string_size;
  }

  p_serial.write(as_bytes(std::string_view(buffer.data(), length)));
}

/**
 * @ingroup Serial
 * @brief Write data to serial buffer and drop return value
 *
 * Only use this with serial ports with infallible write operations, meaning
 * they will never return an error result.
 *
 * @tparam byte_array_t - data array type
 * @param p_serial - serial port to write data to
 * @param p_data - data to be sent over the serial port
 */
template<typename byte_array_t>
void print(v5::serial& p_serial, byte_array_t&& p_data)
{
  p_serial.write(as_bytes(p_data));
}

std::uint64_t future_deadline(hal::v5::steady_clock& p_steady_clock,
                              hal::time_duration p_duration)
{
  using period = decltype(p_duration)::period;
  auto const frequency = p_steady_clock.frequency();
  auto const tick_period = wavelength<period>(static_cast<float>(frequency));
  auto ticks_required = p_duration / tick_period;
  using unsigned_ticks = std::make_unsigned_t<decltype(ticks_required)>;

  if (ticks_required <= 1) {
    ticks_required = 1;
  }

  auto const ticks = static_cast<unsigned_ticks>(ticks_required);
  auto const future_timestamp = ticks + p_steady_clock.uptime();

  return future_timestamp;
}
void delay(hal::v5::steady_clock& p_steady_clock, hal::time_duration p_duration)
{
  auto ticks_until_timeout = future_deadline(p_steady_clock, p_duration);
  while (p_steady_clock.uptime() < ticks_until_timeout) {
    continue;
  }
}
class dummy_allow : public hal::can_identifier_filter
{
  void driver_allow(std::optional<u16>) override
  {
  }
};
}  // namespace hal

void application(resource_list&)
{
  using namespace std::literals;
  using namespace hal::literals;

  hal::dummy_allow dummy{};
  auto clock = resource::steady_clock();
  auto manager = resource::can_bus_manager();
  auto console = resource::serial_console(512);
  // Needs to be set to this baud rate to work with the default firmware CAN
  // baud rate.
  manager->baud_rate(1_MHz);
  manager->filter_mode(hal::v5::can_bus_manager::accept::all);
  manager->bus_on();

  auto transceiver = resource::can_transceiver();

  hal::print(*console, "RMD MC-X Smart Servo Application Starting...\n\n"sv);

  constexpr std::uint16_t starting_device_address = 0x140;
  std::uint16_t address_offset = 0;

  while (true) {
    try {
      auto const address = starting_device_address + address_offset;
      hal::print<32>(*console, "Using address: 0x%04X\n", address);
      hal::actuator::rmd_mc_x_v2 mc_x(
        *transceiver, dummy, *clock, 36.0f, address);

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
        motor.drive(20.0_rpm);
        print_feedback();

        hal::print(*console, "motor drive -20rpm\n"sv);
        motor.drive(-20.0_rpm);
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

    address_offset = (address_offset + 1) % 8;
    hal::delay(*clock, 1s);
  }
}
