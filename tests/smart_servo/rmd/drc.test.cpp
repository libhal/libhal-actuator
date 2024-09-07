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

#include <libhal-actuator/smart_servo/rmd/drc.hpp>

#include <libhal-mock/can.hpp>
#include <libhal-mock/steady_clock.hpp>
#include <libhal-util/can.hpp>
#include <libhal-util/enum.hpp>

#include <boost/ut.hpp>

namespace hal::actuator {
namespace {
constexpr can::id_t expected_id = 0x140;
constexpr can::id_t expected_gear_ratio = 6.0f;

template<size_t N>
constexpr auto prefilled_messages(hal::byte p_command_byte = 0x00)
{
  std::array<can::message_t, N> prefilled_expected_messages{};
  for (auto& message : prefilled_expected_messages) {
    message.id = expected_id;
    message.length = 8;
    message.payload.fill(0x00);
    message.payload[0] = p_command_byte;
  }
  return prefilled_expected_messages;
};

std::queue<std::uint64_t> create_queue()
{
  std::queue<std::uint64_t> new_queue;
  for (std::uint64_t i = 0; i < 255; i++) {
    new_queue.push(i);
  }
  return new_queue;
}

struct rmd_responder : public hal::can
{
  /**
   * @brief Reset spy information for functions
   *
   */
  void reset()
  {
    spy_configure.reset();
    spy_send.reset();
    spy_bus_on.reset();
  }

  /// Spy handler for hal::can::configure()
  spy_handler<settings> spy_configure;
  /// Spy handler for hal::can::send()
  spy_handler<message_t> spy_send;
  /// Spy handler for hal::can::bus_on() will always have content of "true"
  spy_handler<bool> spy_bus_on;
  /// Spy handler for hal::can::on_receive()
  hal::callback<handler> m_on_receive = [](message_t const&) {};

private:
  void driver_configure(settings const& p_settings) override
  {
    return spy_configure.record(p_settings);
  }

  void driver_bus_on() override
  {
    return spy_bus_on.record(true);
  }

  void driver_send(message_t const& p_message) override
  {
    spy_send.record(p_message);
    m_on_receive(p_message);
  }

  void driver_on_receive(hal::callback<handler> p_handler) override
  {
    m_on_receive = p_handler;
  }
};
}  // namespace

boost::ut::suite test_rmd_drc = [] {
  using namespace boost::ut;
  using namespace std::literals;

  "rmd_drc::create()"_test = []() {
    // Setup
    rmd_responder mock_can;
    mock::steady_clock mock_steady;
    auto queue = create_queue();
    mock_steady.set_uptimes(queue);
    mock_steady.set_frequency(1.0_MHz);
    hal::can_router router(mock_can);
    auto expected = prefilled_messages<2>();
    expected[0].payload[0] = hal::value(rmd_drc::system::off);
    expected[1].payload[0] = hal::value(rmd_drc::system::running);

    // Exercise
    rmd_drc driver(router, mock_steady, expected_gear_ratio, expected_id);

    // Verify
    expect(that % 2 == mock_can.spy_send.call_history().size());
    expect(that % expected[0] == mock_can.spy_send.history<0>(0));
    expect(that % expected[1] == mock_can.spy_send.history<0>(1));
  };

  "rmd_drc::create() failure"_test = []() {
    // Setup
    rmd_responder mock_can;
    mock::steady_clock mock_steady;
    auto queue = create_queue();
    mock_steady.set_uptimes(queue);
    mock_steady.set_frequency(1.0_MHz);
    hal::can_router router(mock_can);
    mock_can.spy_send.trigger_error_on_call(
      1, []() { throw hal::io_error(nullptr); });

    constexpr can::message_t expected1{
      .id = expected_id,
      .payload = { hal::value(rmd_drc::system::off),
                   0x00,
                   0x00,
                   0x00,
                   0x00,
                   0x00,
                   0x00,
                   0x00 },
      .length = 8,
    };

    // Exercise
    expect(throws<hal::io_error>([&]() {
      rmd_drc driver(router, mock_steady, expected_gear_ratio, expected_id);
    }));

    // Verify
    expect(that % 1 == mock_can.spy_send.call_history().size());
    expect(that % expected1 == mock_can.spy_send.history<0>(0));
  };

  "rmd_drc::velocity_control()"_test = []() {
    // Setup
    rmd_responder mock_can;
    mock::steady_clock mock_steady;
    auto queue = create_queue();
    mock_steady.set_uptimes(queue);
    mock_steady.set_frequency(1.0_MHz);
    hal::can_router router(mock_can);
    rmd_drc driver(router, mock_steady, expected_gear_ratio, expected_id);
    mock_can.reset();
    auto expected = prefilled_messages<7>(hal::value(rmd_drc::actuate::speed));
    std::array injected_rpm{
      0.0_rpm, 10.0_rpm, 10.0_rpm, 123.0_rpm, 0.0_rpm, 1024.0_rpm,
    };

    using payload_t = decltype(expected[0].payload);

    // Setup: values verified to be correct
    expected[1].payload = payload_t{
      0xa2, 0x0, 0x0, 0x0, 0xa0, 0x8c, 0x0, 0x0,
    };
    expected[2].payload = payload_t{
      0xa2, 0x0, 0x0, 0x0, 0xa0, 0x8c, 0x0, 0x0,
    };
    expected[3].payload = payload_t{
      0xa2, 0x0, 0x0, 0x0, 0xb0, 0xc1, 0x6, 0x0,
    };
    expected[4].payload = payload_t{
      0xa2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    };
    expected[5].payload = payload_t{
      0xa2, 0x0, 0x0, 0x0, 0x0, 0x40, 0x38, 0x0,
    };

    // Exercise
    driver.velocity_control(injected_rpm[0]);
    driver.velocity_control(injected_rpm[1]);
    driver.velocity_control(injected_rpm[2]);
    driver.velocity_control(injected_rpm[3]);
    driver.velocity_control(injected_rpm[4]);
    driver.velocity_control(injected_rpm[5]);

    // Verify
    expect(that % 6 == mock_can.spy_send.call_history().size());
    expect(that % expected[0] == mock_can.spy_send.history<0>(0));
    expect(that % expected[1] == mock_can.spy_send.history<0>(1));
    expect(that % expected[2] == mock_can.spy_send.history<0>(2));
    expect(that % expected[3] == mock_can.spy_send.history<0>(3));
    expect(that % expected[4] == mock_can.spy_send.history<0>(4));
    expect(that % expected[5] == mock_can.spy_send.history<0>(5));
  };

  "rmd_drc::position_control()"_test = []() {
    // Setup
    rmd_responder mock_can;
    mock::steady_clock mock_steady;
    auto queue = create_queue();
    mock_steady.set_uptimes(queue);
    mock_steady.set_frequency(1.0_MHz);
    hal::can_router router(mock_can);
    rmd_drc driver(router, mock_steady, expected_gear_ratio, expected_id);
    mock_can.reset();
    auto expected =
      prefilled_messages<6>(hal::value(rmd_drc::actuate::position_2));
    using payload_t = decltype(expected[0].payload);

    // Setup: values verified to be correct
    expected[0].payload = payload_t{
      0xa4, 0x0, 0x68, 0x1, 0x0, 0x0, 0x0, 0x0,
    };
    expected[1].payload = payload_t{
      0xa4, 0x0, 0x68, 0x1, 0x78, 0x69, 0x0, 0x0,
    };
    expected[2].payload = payload_t{
      0xa4, 0x0, 0x68, 0x1, 0xf0, 0xd2, 0x0, 0x0,
    };
    expected[3].payload = payload_t{
      0xa4, 0x0, 0x68, 0x1, 0x20, 0x1c, 0x0, 0x0,
    };
    expected[4].payload = payload_t{
      0xa4, 0x0, 0x68, 0x1, 0xd8, 0xdc, 0xff, 0xff,
    };
    expected[5].payload = payload_t{
      0xa4, 0x0, 0x68, 0x1, 0x40, 0xc6, 0xf9, 0xff,
    };

    // Exercise
    driver.position_control(0.0_deg, 10.0_rpm);
    driver.position_control(45.0_deg, 10.0_rpm);
    driver.position_control(90.0_deg, 10.0_rpm);
    driver.position_control(12.0_deg, 10.0_rpm);
    driver.position_control(-15.0_deg, 10.0_rpm);
    driver.position_control(-680.0_deg, 10.0_rpm);

    // Verify
    expect(that % expected[0] == mock_can.spy_send.history<0>(0));
    expect(that % expected[1] == mock_can.spy_send.history<0>(1));
    expect(that % expected[2] == mock_can.spy_send.history<0>(2));
    expect(that % expected[3] == mock_can.spy_send.history<0>(3));
    expect(that % expected[4] == mock_can.spy_send.history<0>(4));
    expect(that % expected[5] == mock_can.spy_send.history<0>(5));
  };

  "rmd_drc::feedback_request()"_test = []() {
    // Setup
    rmd_responder mock_can;
    mock::steady_clock mock_steady;
    auto queue = create_queue();
    mock_steady.set_uptimes(queue);
    mock_steady.set_frequency(1.0_MHz);
    hal::can_router router(mock_can);
    rmd_drc driver(router, mock_steady, expected_gear_ratio, expected_id);
    mock_can.reset();

    auto expected = prefilled_messages<3>();

    expected[0].payload[0] = hal::value(rmd_drc::read::multi_turns_angle);
    expected[1].payload[0] =
      hal::value(rmd_drc::read::status_1_and_error_flags);
    expected[2].payload[0] = hal::value(rmd_drc::read::status_2);

    // Exercise
    driver.feedback_request(rmd_drc::read::multi_turns_angle);
    driver.feedback_request(rmd_drc::read::status_1_and_error_flags);
    driver.feedback_request(rmd_drc::read::status_2);

    // Verify
    expect(that % 3 == mock_can.spy_send.call_history().size());
    expect(that % expected[0] == mock_can.spy_send.history<0>(0));
    expect(that % expected[1] == mock_can.spy_send.history<0>(1));
    expect(that % expected[2] == mock_can.spy_send.history<0>(2));
  };

  "rmd_drc::system_control()"_test = []() {
    // Setup
    rmd_responder mock_can;
    mock::steady_clock mock_steady;
    auto queue = create_queue();
    mock_steady.set_uptimes(queue);
    mock_steady.set_frequency(1.0_MHz);
    hal::can_router router(mock_can);
    rmd_drc driver(router, mock_steady, expected_gear_ratio, expected_id);
    mock_can.reset();

    auto expected = prefilled_messages<4>();

    expected[0].payload[0] = hal::value(rmd_drc::system::clear_error_flag);
    expected[1].payload[0] = hal::value(rmd_drc::system::off);
    expected[2].payload[0] = hal::value(rmd_drc::system::stop);
    expected[3].payload[0] = hal::value(rmd_drc::system::running);

    // Exercise
    driver.system_control(rmd_drc::system::clear_error_flag);
    driver.system_control(rmd_drc::system::off);
    driver.system_control(rmd_drc::system::stop);
    driver.system_control(rmd_drc::system::running);

    // Verify
    expect(that % 4 == mock_can.spy_send.call_history().size());
    expect(that % expected[0] == mock_can.spy_send.history<0>(0));
    expect(that % expected[1] == mock_can.spy_send.history<0>(1));
    expect(that % expected[2] == mock_can.spy_send.history<0>(2));
    expect(that % expected[3] == mock_can.spy_send.history<0>(3));
  };

  "rmd_drc::operator() update feedback status_2 "_test = []() {
    // Setup
    rmd_responder mock_can;
    mock::steady_clock mock_steady;
    auto queue = create_queue();
    mock_steady.set_uptimes(queue);
    mock_steady.set_frequency(1.0_MHz);
    hal::can_router router(mock_can);
    rmd_drc driver(router, mock_steady, expected_gear_ratio, expected_id);

    auto expected = prefilled_messages<1>();
    expected[0].payload[0] = hal::value(rmd_drc::read::status_2);
    expected[0].payload[1] = 0x11;  // temperature
    expected[0].payload[2] = 0x22;  // current low byte
    expected[0].payload[3] = 0x33;  // current high byte
    expected[0].payload[4] = 0x44;  // speed low byte
    expected[0].payload[5] = 0x55;  // speed high byte
    expected[0].payload[6] = 0x66;  // encoder low byte
    expected[0].payload[7] = 0x77;  // encoder high byte

    // Exercise
    driver(expected[0]);

    // Verify
    expect(that % 0x11 == driver.feedback().raw_motor_temperature);
    expect(that % 0x3322 == driver.feedback().raw_current);
    expect(that % 0x5544 == driver.feedback().raw_speed);
    expect(that % 0x7766 == driver.feedback().encoder);
  };

  "rmd_drc::feedback().current() "_test = []() {};

  "hal::make_<interface>()"_test = []() {
    // Setup
    rmd_responder mock_can;
    mock::steady_clock mock_steady;
    auto queue = create_queue();
    mock_steady.set_uptimes(queue);
    mock_steady.set_frequency(1.0_MHz);
    hal::can_router router(mock_can);
    rmd_drc driver(router, mock_steady, expected_gear_ratio, expected_id);

    // Exercise
    // Verify
    auto motor = hal::make_motor(driver, 100.0_rpm);
    auto servo = hal::make_servo(driver, 100.0_rpm);
    auto temp = hal::make_temperature_sensor(driver);
    auto rotation = hal::make_rotation_sensor(driver);
  };
};
}  // namespace hal::actuator
