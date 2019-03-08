# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

macro(ros2_create_keystore)
  # ros2_create_keystore()
  #
  # SECURITY (cmake arg) if not define or OFF, will not generate key/keystores
  # ROS_SECURITY_ROOT_DIRECTORY (env variable) will the location of the keystore
  if(NOT SECURITY)
    return()
  endif()
  find_program(PROGRAM ros2)
  if(DEFINED ENV{ROS_SECURITY_ROOT_DIRECTORY})
    set(SECURITY_KEYSTORE $ENV{ROS_SECURITY_ROOT_DIRECTORY})
  else()
    set(SECURITY_KEYSTORE ${DEFAULT_KEYSTORE})
  endif()
  message(STATUS "Keystore located at ${SECURITY_KEYSTORE}")
  if(NOT EXISTS ${SECURITY_KEYSTORE})
    message(STATUS "Creating keystore directory")
    file(MAKE_DIRECTORY ${SECURITY_KEYSTORE})
  endif()

  # Check to see if the security keystore already has already been created
  file(GLOB RESULT "${SECURITY_KEYSTORE}/")
  list(LENGTH RESULT RES_LEN)
  if(${RES_LEN} EQUAL 0)
    message(STATUS "Creating keystore directory")
    execute_process(
      COMMAND ${PROGRAM} security create_keystore ${SECURITY_KEYSTORE}
    )
  endif()
endmacro()
