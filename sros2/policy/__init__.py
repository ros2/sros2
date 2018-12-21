# Copyright 2018-2019 Open Source Robotics Foundation, Inc.
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

import os

import pkg_resources


def get_policy_schema(name):
    return pkg_resources.resource_filename(
        package_or_requirement='sros2',
        resource_name=os.path.join('policy', 'schemas', name))


def get_transport_default(transport, name):
    return pkg_resources.resource_filename(
        package_or_requirement='sros2',
        resource_name=os.path.join('policy', 'defaults', transport, name))


def get_transport_schema(transport, name):
    return pkg_resources.resource_filename(
        package_or_requirement='sros2',
        resource_name=os.path.join('policy', 'schemas', transport, name))


def get_transport_template(transport, name):
    return pkg_resources.resource_filename(
        package_or_requirement='sros2',
        resource_name=os.path.join('policy', 'templates', transport, name))
