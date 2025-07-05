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

#include <libhal-util/serial.hpp>
#include <libhal/can.hpp>
#include <memory_resource>
#include <optional>

#include <libhal-expander/canusb.hpp>
#include <libhal-mac/console.hpp>
#include <libhal-mac/serial.hpp>
#include <libhal-mac/steady_clock.hpp>
#include <libhal-util/can.hpp>
#include <libhal/error.hpp>
#include <libhal/pointers.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/units.hpp>

#include <resource_list.hpp>

namespace resources {

void reset()
{
  exit(-1);
}

void sleep(hal::time_duration p_duration)
{
  std::this_thread::sleep_for(p_duration);
}

std::pmr::polymorphic_allocator<> driver_allocator()
{
  return std::pmr::new_delete_resource();
}

hal::v5::strong_ptr<hal::serial> console()
{
  auto console = hal::mac::console_serial::create(driver_allocator(), 1024);
  return hal::make_serial_converter(resources::driver_allocator(), console);
}

hal::v5::strong_ptr<hal::steady_clock> clock()
{
  return hal::v5::make_strong_ptr<hal::mac::legacy_steady_clock>(
    driver_allocator());
}

hal::v5::strong_ptr<hal::output_pin> status_led()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::strong_ptr<hal::i2c> i2c()
{
  throw hal::operation_not_supported(nullptr);
}

hal::v5::optional_ptr<hal::expander::canusb> can_usb;

void initialize_can_usb()
{
  if (can_usb) {
    return;
  }
  using namespace std::literals;
  using namespace hal::literals;

  // NOTE: Change this to the USB serial port path...
  constexpr auto usb_serial_path = "/dev/tty.usbserial-59760073631";
  auto serial = hal::mac::serial::create(
    driver_allocator(), usb_serial_path, 1024, { .baud_rate = 115200 });

  // Assert DTR and RTS
  serial->set_control_signals(true, true);
  std::this_thread::sleep_for(500ms);
  // De-activate RTS (boot) line
  serial->set_rts(false);
  std::this_thread::sleep_for(500ms);
  // De-activate DTR (reset) line to reset device
  serial->set_dtr(false);
  std::this_thread::sleep_for(500ms);

  can_usb =
    hal::expander::canusb::create(resources::driver_allocator(), serial);
}

hal::v5::strong_ptr<hal::can_bus_manager> can_bus_manager()
{
  using namespace hal::literals;

  initialize_can_usb();

  auto manager =
    hal::acquire_can_bus_manager(resources::driver_allocator(), can_usb);

  manager->baud_rate(1_MHz);
  manager->filter_mode(hal::v5::can_bus_manager::accept::all);
  manager->bus_on();
  auto adapter = hal::v5::make_strong_ptr<hal::can_bus_manager_adapter>(
    resources::driver_allocator(), manager);

  return adapter;
}

hal::v5::strong_ptr<hal::can_transceiver> can_transceiver()
{
  return hal::acquire_can_transceiver(
    resources::driver_allocator(), can_usb, 32);
}

hal::v5::strong_ptr<hal::can_identifier_filter> can_identifier_filter()
{
  class dummy_allow : public hal::can_identifier_filter
  {
    void driver_allow(std::optional<hal::u16>) override
    {
    }
  };
  return hal::v5::make_strong_ptr<dummy_allow>(resources::driver_allocator());
}

hal::v5::strong_ptr<hal::pwm> pwm()
{
  throw hal::operation_not_supported(nullptr);
}
}  // namespace resources

void initialize_platform()
{
  // do nothing
}
