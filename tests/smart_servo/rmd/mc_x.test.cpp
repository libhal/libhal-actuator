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

#include <libhal-actuator/smart_servo/rmd/mc_x.hpp>

#include <libhal-mock/can.hpp>
#include <libhal-mock/steady_clock.hpp>
#include <libhal-util/enum.hpp>

#include <boost/ut.hpp>

namespace hal::actuator {
boost::ut::suite test_rmd_mc_x = [] {
  using namespace boost::ut;
  using namespace std::literals;
  using namespace hal::literals;

  "hal::actuator::rmd_mc_x::rmd_mc_x()"_test = []() {};
};
}  // namespace hal::actuator