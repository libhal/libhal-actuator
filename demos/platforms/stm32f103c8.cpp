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

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-armcortex/startup.hpp>
#include <libhal-armcortex/system_control.hpp>

#include <libhal-stm32f1/can.hpp>
#include <libhal-stm32f1/clock.hpp>
#include <libhal-stm32f1/constants.hpp>
#include <libhal-stm32f1/output_pin.hpp>
#include <libhal-stm32f1/uart.hpp>

#include <resource_list.hpp>

void initialize_platform(resource_list& p_resources)
{
  using namespace hal::literals;

  // Set the MCU to the maximum clock speed
  hal::stm32f1::maximum_speed_using_internal_oscillator();

  static hal::cortex_m::dwt_counter counter(
    hal::stm32f1::frequency(hal::stm32f1::peripheral::cpu));

  static hal::stm32f1::uart uart1(hal::port<1>,
                                  hal::buffer<128>,
                                  hal::serial::settings{
                                    .baud_rate = 115200,
                                  });

  static hal::stm32f1::output_pin led('C', 13);

  p_resources.reset = +[]() { hal::cortex_m::reset(); };
  p_resources.console = &uart1;
  p_resources.clock = &counter;
  p_resources.status_led = &led;

  if constexpr (use_can_v1) {
    static hal::stm32f1::can driver({}, hal::stm32f1::can_pins::pb9_pb8);
    p_resources.can = &driver;
  } else {
    using namespace std::chrono_literals;
    static std::array<hal::can_message, 4> receive_buffer{};
    static hal::stm32f1::can_peripheral_manager can(
      100_kHz, counter, 1ms, hal::stm32f1::can_pins::pb9_pb8);
    static auto can_transceiver = can.acquire_transceiver(receive_buffer);
    p_resources.can_transceiver = &can_transceiver;
    static auto can_bus_manager = can.acquire_bus_manager();
    p_resources.can_bus_manager = &can_bus_manager;
    static auto can_identifier_filter = can.acquire_identifier_filter();
    p_resources.can_identifier_filter = &can_identifier_filter.filter[0];
  }
}
