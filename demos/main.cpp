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

#include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>

#include <resource_list.hpp>

resource_list resources{};

[[noreturn]] void terminate_handler() noexcept
{
  if (not resources.status_led && not resources.clock) {
    // spin here and wait for a debugger to be connected...
    while (true) {
      continue;
    }
  }

  // Safe access via ::value() is not necessary here since we have already
  // checked the resources above.
  auto& led = **resources.status_led;
  auto& clock = **resources.clock;

  if (resources.console) {
    hal::print(*resources.console.value(),
               "Application Terminated!! Status LED: TERMINATE PATTERN\n");
  }

  // Otherwise, blink the led in a pattern...
  while (true) {
    using namespace std::chrono_literals;
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
// test
int main()
{
  hal::set_terminate(terminate_handler);
  initialize_platform(resources);

  try {
    application(resources);
  } catch (std::bad_optional_access const& e) {
    if (resources.console) {
      hal::print(*resources.console.value(),
                 "A resource required by the application was not available!\n"
                 "Calling terminate!\n");
    }
  }  // Allow any other exceptions to terminate the application

  // Terminate if the code reaches this point.
  std::terminate();
}
