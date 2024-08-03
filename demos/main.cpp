// Copyright 2024 Khalil Estell
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

#include <libhal-exceptions/control.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>

#include "resource_list.hpp"

resource_list resources{};

[[noreturn]] void terminate_handler() noexcept
{
  using namespace std::chrono_literals;

  auto& led = *resources.led;
  auto& clock = *resources.clock;

  while (true) {
    led.level(false);
    hal::delay(clock, 100ms);
    led.level(true);
    hal::delay(clock, 100ms);
    led.level(false);
    hal::delay(clock, 100ms);
    led.level(true);
    hal::delay(clock, 1000ms);
  }
}

int main()
{
  // Set terminate routine...
  hal::set_terminate(terminate_handler);

  try {
    resources = initialize_platform();
  } catch (...) {
    // Catch all exceptions preventing terminate from
    hal::halt();
  }

  application(resources);
  resources.reset();
  return 0;
}
