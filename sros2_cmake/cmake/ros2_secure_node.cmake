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

macro(ros2_secure_node)
  # ros2_secure_node(NODES <node_1> <node_2>...<node_n>)
  #
  # NODES (macro multi-arg) takes the node names for which keys will be generated
  # SECURITY (cmake arg) if not defined or OFF, will not generate key/keystores
  # ROS_SECURITY_ROOT_DIRECTORY (env variable) will the location of the keystore
  # POLICY_FILE (cmake arg) if defined, will compile policies by node name into the access private certificates (e.g POLICY_FILE=/etc/policies/<policy.xml>, Generate: <node_name> /etc/policies/<policy.xml>)
  if(NOT SECURITY)
    message(STATUS "Not generating security files")
    return()
  endif()
  find_program(PROGRAM ros2)
  ros2_create_keystore()
  set(multiValueArgs NODES)
  cmake_parse_arguments(ros2_secure_node "" "" "${multiValueArgs}" ${ARGN})
  foreach(node ${ros2_secure_node_NODES})
    set(create_key_command ${PROGRAM} security create_key ${SECURITY_KEYSTORE} ${node})
    message(STATUS "Executing: ${create_key_command}")
    execute_process(COMMAND ${create_key_command})
    if(POLICY_FILE)
      if(EXISTS ${POLICY_FILE})
        set(policy ${POLICY_FILE})
        set(create_permission_command ${PROGRAM} security create_permission ${SECURITY_KEYSTORE} ${node} ${policy})
        message(STATUS "Executing: ${create_permission_command}")
        execute_process(
          COMMAND ${create_permission_command}
          RESULT_VARIABLE POLICY_RESULT
          ERROR_VARIABLE POLICY_ERROR
        )
        if(NOT ${POLICY_RESULT} EQUAL 0)
          message("Unable to generate policy for ${node} in ${policy}")
          message("${POLICY_ERROR}")
        endif()
      endif()
    endif()
  endforeach()
endmacro()

