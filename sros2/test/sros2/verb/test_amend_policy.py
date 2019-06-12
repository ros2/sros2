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

from sros2.api import NodeName
from sros2.verb.amend_policy import getFQN


def test_getFQN():
    node_name = NodeName('node', '/ns', '/ns/node')
    # Relative name
    assert getFQN(node_name, 'chatter') == '/ns/chatter'
    assert getFQN(node_name, 'foo/chatter') == '/ns/foo/chatter'

    # Private name
    assert getFQN(node_name, '~chatter') == '/ns/node/chatter'
    assert getFQN(node_name, '~foo/chatter') == '/ns/node/foo/chatter'

    # Fully qualified name
    assert getFQN(node_name, '/foo/chatter') == '/foo/chatter' 
    
def test_policyAllowsEvent():
    node_name = NodeName('talker', '/', '/talker')
    policy_document = 
