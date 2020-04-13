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

from sros2.policy import load_policy

from . import _key, _keystore, _permission, _policy, _utilities


def generate_artifacts(keystore_path=None, identity_names=[], policy_files=[]):
    if keystore_path is None:
        keystore_path = _utilities.get_keystore_path_from_env()
        if keystore_path is None:
            return False
    if not _keystore.is_valid_keystore(keystore_path):
        print('%s is not a valid keystore, creating new keystore' % keystore_path)
        _keystore.create_keystore(keystore_path)

    # create keys for all provided identities
    for identity in identity_names:
        if not _key.create_key(keystore_path, identity):
            return False
    for policy_file in policy_files:
        policy_tree = load_policy(policy_file)
        enclaves_element = policy_tree.find('enclaves')
        for enclave in enclaves_element:
            identity_name = enclave.get('path')
            if identity_name not in identity_names:
                if not _key.create_key(keystore_path, identity_name):
                    return False
            policy_element = _policy.get_policy_from_tree(identity_name, policy_tree)
            _permission.create_permissions_from_policy_element(
                keystore_path, identity_name, policy_element)
    return True
