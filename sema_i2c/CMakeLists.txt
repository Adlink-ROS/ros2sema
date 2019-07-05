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
project(sema_i2c)

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
find_package(sensor_msgs REQUIRED)
find_package(sema_vendor REQUIRED)
if (sema_vendor_FOUND)
  find_package(libsema REQUIRED)
endif()

# Add include directories
include_directories(
  # ${libsema_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include/neuron_iic
  )

# Source Code
add_executable(neuron_demo_i2c
  src/main.cpp
  src/neuronIIc.cpp
  src/neuronIIcNode.cpp)
target_link_libraries(neuron_demo_i2c semaeapi)
ament_target_dependencies(neuron_demo_i2c
  rclcpp std_msgs
  sensor_msgs)

install(TARGETS
  neuron_demo_i2c
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
