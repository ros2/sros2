# Copyright 2016-2019 Open Source Robotics Foundation, Inc.
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

from copy import deepcopy

from lxml import etree

from sros2.policy import load_policy


def get_policy(name, policy_file_path):
    policy_tree = load_policy(policy_file_path)
    return get_policy_from_tree(name, policy_tree)


def get_policy_from_tree(name, policy_tree):
    enclave_element = policy_tree.find(
        path=f'enclaves/enclave[@path="{name}"]')
    if enclave_element is None:
        raise RuntimeError(f'unable to find enclave "{name}"')
    enclaves_element = etree.Element('enclaves')
    enclaves_element.append(deepcopy(enclave_element))
    policy_element = etree.Element('policy')
    policy_element.append(enclaves_element)
    return policy_element
