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
    AmendPolicyVerb,
    Event,
    EventPermission,
    get_event_permission_for_profile,
    get_FQN
)

TEST_POLICY = """<policy version="0.1.0">
  <profile ns='/ns' node='node'>
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
</policy>
"""


def test_get_FQN():
    node_name = NodeName('node', '/ns', '/ns/node')
    # Relative name
    assert get_FQN(node_name, 'chatter') == '/ns/chatter'
    assert get_FQN(node_name, 'foo/chatter') == '/ns/foo/chatter'

    # Private name
    assert get_FQN(node_name, '~chatter') == '/ns/node/chatter'
    assert get_FQN(node_name, '~foo/chatter') == '/ns/node/foo/chatter'

    # Fully qualified name
    assert get_FQN(node_name, '/foo/chatter') == '/foo/chatter'


def test_get_event_permission_for_profile():
    node_name = NodeName('node', '/ns', '/ns/node')
    # Get profile
    policy = etree.fromstring(TEST_POLICY)
    profile = policy[0]

    assert EventPermission.ALLOW == get_event_permission_for_profile(
      profile, Event(node_name, 'topic', 'subscribe', 'parameter_events'))
    assert EventPermission.ALLOW == get_event_permission_for_profile(
      profile, Event(node_name, 'topic', 'publish', 'parameter_events'))
    assert EventPermission.ALLOW == get_event_permission_for_profile(
      profile, Event(node_name, 'service', 'reply', '~get_parameters'))
    assert EventPermission.NOT_SPECIFIED == get_event_permission_for_profile(
      profile, Event(node_name, 'topic', 'publish', 'not_a_topic'))
    assert EventPermission.DENY == get_event_permission_for_profile(
      profile, Event(node_name, 'topic', 'publish', 'denied_topic'))


def test_get_permission_groups():
    policy = etree.fromstring(TEST_POLICY)
    profile = policy[0]

    # Permission exists
    assert len(AmendPolicyVerb.get_permission_groups(
            profile, 'topic', 'subscribe', 'parameter_events')) == 0

    # Permission doesn't exist
    assert not AmendPolicyVerb.get_permission_groups(
            profile, 'topic', 'publish', 'not_a_topic')


def test_create_permission():
    node_name = NodeName('node', '/ns', '/ns/node')
    # Expression is private to node
    event = Event(node_name, 'topic', 'publish', '/ns/node/parameter_events')
    permission_string = b'<topic>~parameter_events</topic>'
    assert permission_string == etree.tostring(AmendPolicyVerb.create_permission(event))

    # Expression is in node namespace
    event = Event(node_name, 'topic', 'publish', '/ns/parameter_events')
    permission_string = b'<topic>parameter_events</topic>'
    assert permission_string == etree.tostring(AmendPolicyVerb.create_permission(event))

    # Expression is in root namespace with node
    root_node_name = NodeName('node', '/', '/node')
    event = Event(root_node_name, 'topic', 'publish', '/parameter_events')
    permission_string = b'<topic>parameter_events</topic>'
    assert permission_string == etree.tostring(AmendPolicyVerb.create_permission(event))

    # Expression is in another namespace
    event = Event(node_name, 'topic', 'publish', '/other_ns/parameter_events')
    permission_string = b'<topic>/other_ns/parameter_events</topic>'
    assert permission_string == etree.tostring(AmendPolicyVerb.create_permission(event))


def elements_equal(e1, e2):
    if e1.tag != e2.tag:
        return False
    if e1.text != e2.text:
        return False
    if e1.tail != e2.tail:
        return False
    if e1.attrib != e2.attrib:
        return False
    if len(e1) != len(e2):
        return False
    return all(elements_equal(c1, c2) for c1, c2 in zip(e1, e2))


def test_add_permission():
    parser = etree.XMLParser(remove_blank_text=True)

    # No profile for node
    no_profile_policy_before = """<policy></policy>"""
    no_profile_policy_after = """<policy>
  <profiles>
    <profile ns="/ns" node="node">
      <topics publish="ALLOW">
        <topic>pub_topic</topic>
      </topics>
    </profile>
  </profiles>
</policy>"""
    node_name = NodeName('node', '/ns', '/ns/node')
    event = Event(node_name, 'topic', 'publish', '/ns/pub_topic')
    policy = etree.XML(no_profile_policy_before, parser=parser)
    AmendPolicyVerb.add_permission(policy, event, EventPermission.ALLOW)
    assert elements_equal(etree.XML(no_profile_policy_after, parser=parser), policy)

    # Profile exists, no permission group with type
    no_perm_group_policy_before = (
        '<policy>'
        '<profiles>'
        '<profile ns="/ns" node="node">'
        '</profile>'
        '</profiles>'
        '</policy>'
    )

    no_perm_group_policy_after = (
        '<policy>'
        '<profiles>'
        '<profile ns="/ns" node="node">'
        '<topics publish="ALLOW">'
        '<topic>pub_topic</topic>'
        '</topics>'
        '</profile>'
        '</profiles>'
        '</policy>'
    )
    node_name = NodeName('node', '/ns', '/ns/node')
    event = Event(node_name, 'topic', 'publish', '/ns/pub_topic')
    policy = etree.XML(no_perm_group_policy_before, parser=parser)
    AmendPolicyVerb.add_permission(policy, event, EventPermission.ALLOW)

    assert elements_equal(etree.XML(no_perm_group_policy_after, parser=parser), policy)
    print(etree.tostring(etree.XML(no_perm_group_policy_after, parser=parser)))
    print(etree.tostring(policy))

    # Profile ,permission group exits with type, no rule type
    no_rule_type_policy_before = (
        '<policy>'
        '<profiles>'
        '<profile ns="/ns" node="node">'
        '<topics subscribe="ALLOW">'
        '<topic>topic1</topic>'
        '</topics>'
        '</profile>'
        '</profiles>'
        '</policy>'
    )

    no_rule_type_policy_after = (
        '<policy>'
        '<profiles>'
        '<profile ns="/ns" node="node">'
        '<topics subscribe="ALLOW">'
        '<topic>topic1</topic>'
        '</topics>'
        '<topics publish="ALLOW">'
        '<topic>topic1</topic>'
        '</topics>'
        '</profile>'
        '</profiles>'
        '</policy>'
    )

    node_name = NodeName('node', '/ns', '/ns/node')
    event = Event(node_name, 'topic', 'publish', '/ns/topic1')
    policy = etree.XML(no_rule_type_policy_before, parser=parser)
    AmendPolicyVerb.add_permission(policy, event, EventPermission.ALLOW)
    assert elements_equal(etree.XML(no_rule_type_policy_after, parser=parser), policy)

    # Profile, permission group with type, rule type exist, wrong rule qualifier
    no_rule_qual_policy_before = (
        '<policy>'
        '<profiles>'
        '<profile ns="/ns" node="node">'
        '<topics subscribe="ALLOW">'
        '<topic>topic1</topic>'
        '</topics>'
        '</profile>'
        '</profiles>'
        '</policy>'
    )

    no_rule_qual_policy_after = (
        '<policy>'
        '<profiles>'
        '<profile ns="/ns" node="node">'
        '<topics subscribe="ALLOW">'
        '<topic>topic1</topic>'
        '</topics>'
        '<topics subscribe="DENY">'
        '<topic>topic2</topic>'
        '</topics>'
        '</profile>'
        '</profiles>'
        '</policy>'
    )

    node_name = NodeName('node', '/ns', '/ns/node')
    event = Event(node_name, 'topic', 'subscribe', '/ns/topic2')
    policy = etree.XML(no_rule_qual_policy_before, parser=parser)
    AmendPolicyVerb.add_permission(policy, event, EventPermission.DENY)
    assert elements_equal(etree.XML(no_rule_qual_policy_after, parser=parser), policy)

    # Profile permission group with type, rule type and rule qualifier exist
    all_exist_policy_before = (
        '<policy>'
        '<profiles>'
        '<profile ns="/ns" node="node">'
        '<topics subscribe="ALLOW">'
        '<topic>topic1</topic>'
        '</topics>'
        '</profile>'
        '</profiles>'
        '</policy>'
    )

    all_exist_policy_after = (
        '<policy>'
        '<profiles>'
        '<profile ns="/ns" node="node">'
        '<topics subscribe="ALLOW">'
        '<topic>topic1</topic>'
        '<topic>topic2</topic>'
        '</topics>'
        '</profile>'
        '</profiles>'
        '</policy>'
    )

    node_name = NodeName('node', '/ns', '/ns/node')
    event = Event(node_name, 'topic', 'subscribe', '/ns/topic2')
    policy = etree.XML(all_exist_policy_before, parser=parser)
    AmendPolicyVerb.add_permission(policy, event, EventPermission.ALLOW)
    assert elements_equal(etree.XML(all_exist_policy_after, parser=parser), policy)
