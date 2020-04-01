# Copyright 2019 Open Source Robotics Foundation Inc.
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

# copied from sros2_cmake/sros2_cmake-extras.cmake

set(DEFAULT_KEYSTORE "${CMAKE_INSTALL_PREFIX}/ros2_security/keystore")

include("${sros2_cmake_DIR}/ros2_secure_node.cmake")

# register ament_package() hook for security policies once.
macro(_sros2_cmake_register_package_hook)
  if(NOT DEFINED _SROS2_CMAKE_PACKAGE_HOOK_REGISTERED)
    set(_SROS2_CMAKE_PACKAGE_HOOK_REGISTERED TRUE)

    find_package(ament_cmake_core QUIET REQUIRED)
    ament_register_extension("ament_package" "sros2_cmake"
      "sros2_cmake_package_hook.cmake")
  endif()
endmacro()

include("${sros2_cmake_DIR}/sros2_cmake_install_policies.cmake")

