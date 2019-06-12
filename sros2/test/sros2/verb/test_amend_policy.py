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

from lxml import etree

from sros2.api import NodeName
from sros2.verb.amend_policy import (
    Event,
    EventPermission,
    getEventPermissionForProfile,
    getFQN
)

TEST_POLICY = """<profile ns='/ns' node='node'>
  <topics publish="ALLOW" subscribe="ALLOW" >
    <topic>parameter_events</topic>
  </topics>

  <topics publish="DENY" >
    <topic>denied_topic</topic>
  </topics>

  <services reply="ALLOW" request="ALLOW" >
    <service>~describe_parameters</service>
    <service>~get_parameter_types</service>
    <service>~get_parameters</service>
    <service>~list_parameters</service>
    <service>~set_parameters</service>
    <service>~set_parameters_atomically</service>
  </services>
</profile>
"""


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


def test_getEventPermissionForProfile():
    node_name = NodeName('node', '/ns', '/ns/node')
    # Get profile
    profile = etree.fromstring(TEST_POLICY)

    assert EventPermission.ALLOW == getEventPermissionForProfile(
      profile, Event(node_name, 'topic', 'subscribe', 'parameter_events'))
    assert EventPermission.ALLOW == getEventPermissionForProfile(
      profile, Event(node_name, 'topic', 'publish', 'parameter_events'))
    assert EventPermission.ALLOW == getEventPermissionForProfile(
      profile, Event(node_name, 'service', 'reply', '~get_parameters'))
    assert EventPermission.NOT_SPECIFIED == getEventPermissionForProfile(
      profile, Event(node_name, 'topic', 'publish', 'not_a_topic'))
    assert EventPermission.DENY == getEventPermissionForProfile(
      profile, Event(node_name, 'topic', 'publish', 'denied_topic'))
