# Copyright 2024 - 2025 Khalil Estell and the libhal contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from conan import ConanFile


required_conan_version = ">=2.0.14"


class libhal_actuator_conan(ConanFile):
    name = "libhal-actuator"
    license = "Apache-2.0"
    homepage = "https://github.com/libhal/libhal-actuator"
    description = ("A collection of drivers for the actuator")
    topics = ("actuator", "libhal", "driver")
    settings = "compiler", "build_type", "os", "arch"

    python_requires = "libhal-bootstrap/[>=4.3.0 <5]"
    python_requires_extend = "libhal-bootstrap.library"

    def requirements(self):
        self.requires("libhal/[^4.9.0]", transitive_headers=True)
        self.requires("libhal-util/[^5.4.1]", transitive_headers=True)
        self.requires("libhal-canrouter/[^3.0.0]")

    def package_info(self):
        self.cpp_info.libs = ["libhal-actuator"]
        self.cpp_info.set_property("cmake_target_name", "libhal::actuator")
