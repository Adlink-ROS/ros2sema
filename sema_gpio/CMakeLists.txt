# Copyright 2019 ADLINK Technology Ltd. Advanced Robotic Platform Group
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(sema_gpio)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(libsema REQUIRED)

# Add include directories
include_directories(
  # ${libsema_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include/neuron_gpio
  )

# Source Code
add_executable(neuron_demo_gpio
  src/main.cpp
  src/neuronGpioNode.cpp
  src/neuronGpio.cpp)
target_link_libraries(neuron_demo_gpio semaeapi sema sema_hwlinux EApi_1 log4cpp)
ament_target_dependencies(neuron_demo_gpio
  rclcpp
  std_msgs)

add_executable(neuron_omni_io
  src/omni_main.cpp
  src/neuronOmniIoNode.cpp
  src/neuronGpio.cpp)
target_link_libraries(neuron_omni_io semaeapi)
ament_target_dependencies(neuron_omni_io
  rclcpp
  std_msgs)

add_executable(neuron_omni_test
  src/omni_test.cpp)
target_link_libraries(neuron_omni_test)
ament_target_dependencies(neuron_omni_test
  rclcpp
  std_msgs)

add_executable(neuron_omni_test2
  src/omni_test2.cpp
  src/neuronOmniIoNodeTest.hpp
  src/neuronGpio.cpp)
target_link_libraries(neuron_omni_test2 semaeapi)
ament_target_dependencies(neuron_omni_test2
  rclcpp
  std_msgs)

install(TARGETS
  neuron_demo_gpio
  neuron_omni_io
  neuron_omni_test
  neuron_omni_test2
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
