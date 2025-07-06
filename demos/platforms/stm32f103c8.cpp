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

#include <libhal/units.hpp>

#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/system_control.hpp>

#include <libhal-arm-mcu/stm32f1/can.hpp>
#include <libhal-arm-mcu/stm32f1/clock.hpp>
#include <libhal-arm-mcu/stm32f1/constants.hpp>
#include <libhal-arm-mcu/stm32f1/output_pin.hpp>
#include <libhal-arm-mcu/stm32f1/uart.hpp>

#include <libhal-exceptions/control.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/pointers.hpp>

#include <resource_list.hpp>

// Global optionals for exception handling
hal::v5::optional_ptr<hal::steady_clock> clock_ptr;
hal::v5::optional_ptr<hal::output_pin> status_led_ptr;

[[noreturn]] void terminate_handler() noexcept
{
  if (not status_led_ptr && not clock_ptr) {
    // spin here until debugger is connected
    while (true) {
      continue;
    }
  }

  // Otherwise, blink the led in a pattern
  while (true) {
    using namespace std::chrono_literals;
    status_led_ptr->level(false);
    hal::delay(*clock_ptr, 100ms);
    status_led_ptr->level(true);
    hal::delay(*clock_ptr, 100ms);
    status_led_ptr->level(false);
    hal::delay(*clock_ptr, 100ms);
    status_led_ptr->level(true);
    hal::delay(*clock_ptr, 1000ms);
  }
}

void initialize_platform()
{
  using namespace hal::literals;
  hal::set_terminate(terminate_handler);
  // Set the MCU to the maximum clock speed
  hal::stm32f1::maximum_speed_using_internal_oscillator();
}

namespace resources {
using namespace hal::literals;

std::pmr::polymorphic_allocator<> driver_allocator()
{
  static std::array<hal::byte, 1024> driver_memory{};
  static std::pmr::monotonic_buffer_resource resource(
    driver_memory.data(),
    driver_memory.size(),
    std::pmr::null_memory_resource());
  return &resource;
}

void reset()
{
  hal::cortex_m::reset();
}

void sleep(hal::time_duration p_duration)
{
  auto delay_clock = resources::clock();
  hal::delay(*delay_clock, p_duration);
}

hal::v5::strong_ptr<hal::steady_clock> clock()
{
  if (clock_ptr) {
    return clock_ptr;
  }

  clock_ptr = hal::v5::make_strong_ptr<hal::cortex_m::dwt_counter>(
    driver_allocator(), hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));
  return clock_ptr;
}

hal::v5::strong_ptr<hal::serial> console()
{
  return hal::v5::make_strong_ptr<hal::stm32f1::uart>(driver_allocator(),
                                                      hal::port<1>,
                                                      hal::buffer<128>,
                                                      hal::serial::settings{
                                                        .baud_rate = 115200,
                                                      });
}

hal::v5::strong_ptr<hal::output_pin> status_led()
{
  if (status_led_ptr) {
    return status_led_ptr;
  }

  status_led_ptr = hal::v5::make_strong_ptr<hal::stm32f1::output_pin>(
    driver_allocator(), 'C', 13);
  return status_led_ptr;
}

hal::v5::strong_ptr<hal::can_transceiver> can_transceiver()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::can_identifier_filter> can_identifier_filter()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::pwm> pwm()
{
  throw hal::operation_not_supported(nullptr);
}

}  // namespace resources
