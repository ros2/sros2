# Copyright 2019 Open Source Robotics Foundation, Inc.
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

#
# Installed sros2 security policies and register with the ament resource index.
#
# :param ARGN: the policy files to install
# :type ARGN: list of strings
#
macro(sros2_cmake_install_policies)
  if(${ARGC} GREATER 0)
    _sros2_cmake_register_package_hook()
    foreach(_policy_file ${ARGN})
      get_filename_component(_parent_folder "${_policy_file}" DIRECTORY)
      install(
        FILES ${_policy_file}
        DESTINATION "share/${PROJECT_NAME}/${_parent_folder}"
      )
      get_filename_component(_name "${_policy_file}" NAME)
      list(APPEND _sros2_cmake_POLICY_FILES "${_parent_folder}/${_name}")
    endforeach()
  endif()
endmacro()

