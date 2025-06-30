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

#pragma once

#include <optional>

#include <libhal/can.hpp>
#include <libhal/functional.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/pointers.hpp>
#include <libhal/pwm.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>

struct resource_list
{
  hal::callback<void()> reset;
  std::optional<hal::serial*> console{};
  std::optional<hal::steady_clock*> clock{};
  std::optional<hal::output_pin*> status_led{};
  std::optional<hal::can*> can{};
  std::optional<hal::can_transceiver*> can_transceiver{};
  std::optional<hal::can_bus_manager*> can_bus_manager{};
  std::optional<hal::can_identifier_filter*> can_identifier_filter{};
  std::optional<hal::pwm*> pwm{};
};

namespace resource {
hal::v5::strong_ptr<hal::can_transceiver> can_transceiver();
hal::v5::strong_ptr<hal::v5::can_bus_manager> can_bus_manager();
hal::v5::strong_ptr<hal::v5::serial> serial_console(hal::usize p_buffer_size);
hal::v5::strong_ptr<hal::steady_clock> steady_clock();
}  // namespace resource

// Application function is implemented by one of the .cpp files.
void initialize_platform(resource_list& p_resources);
void application(resource_list& p_resources);

constexpr bool use_can_v1 = false;
