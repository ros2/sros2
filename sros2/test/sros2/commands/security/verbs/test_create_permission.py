# Copyright 2020 Canonical Ltd
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

import pathlib

import lxml

import pytest

import rclpy
from ros2cli import cli

import sros2.keystore
from sros2.policy import get_transport_schema


_test_identity = '/talker_listener/talker'


# This fixture will run once for the entire module (as opposed to once per test)
@pytest.fixture(scope='module')
def enclave_dir(tmp_path_factory, test_policy_dir) -> pathlib.Path:
    keystore_dir = tmp_path_factory.mktemp('keystore')

    # First, create the keystore as well as an enclave for the talker
    sros2.keystore.create_keystore(keystore_dir)
    assert keystore_dir.is_dir()
    sros2.keystore.create_enclave(keystore_dir, _test_identity)

    security_files_dir = keystore_dir.joinpath(f'enclaves{_test_identity}')
    assert security_files_dir.is_dir()

    # Now using that keystore, create a permissions file using the sample policy
    policy_file_path = test_policy_dir / 'sample.policy.xml'
    assert cli.main(
        argv=[
            'security', 'create_permission', str(keystore_dir), _test_identity,
            str(policy_file_path)]) == 0

    # Return path to directory containing the identity's files
    return security_files_dir


def test_create_permission(enclave_dir):
    assert enclave_dir.joinpath('permissions.xml').is_file()
    assert enclave_dir.joinpath('permissions.p7s').is_file()

    tree = lxml.etree.parse(str(enclave_dir.joinpath('permissions.xml')))

    # Validate the schema
    permissions_xsd_path = get_transport_schema('dds', 'permissions.xsd')
    permissions_xsd = lxml.etree.XMLSchema(lxml.etree.parse(str(permissions_xsd_path)))
    permissions_xsd.assertValid(tree)

    dds = tree.getroot()
    assert dds.tag == 'dds'

    permissions = list(dds.iterchildren(tag='permissions'))
    assert len(permissions) == 1

    grants = list(permissions[0].iterchildren(tag='grant'))
    assert len(grants) == 1
    assert grants[0].get('name') == _test_identity

    allow_rules = list(grants[0].iterchildren(tag='allow_rule'))
    if (rclpy.get_rmw_implementation_identifier() in
            sros2.keystore._permission._RMW_WITH_ROS_GRAPH_INFO_TOPIC):
        assert len(allow_rules) == 2
    else:
        assert len(allow_rules) == 1

    publish_rules = list(allow_rules[0].iterchildren(tag='publish'))
    assert len(publish_rules) == 1

    subscribe_rules = list(allow_rules[0].iterchildren(tag='subscribe'))
    assert len(subscribe_rules) == 1

    published_topics_set = list(publish_rules[0].iterchildren(tag='topics'))
    assert len(published_topics_set) == 1
    published_topics = [c.text for c in published_topics_set[0].iterchildren(tag='topic')]
    assert len(published_topics) > 0

    subscribed_topics_set = list(subscribe_rules[0].iterchildren(tag='topics'))
    assert len(subscribed_topics_set) == 1
    subscribed_topics = [c.text for c in subscribed_topics_set[0].iterchildren(tag='topic')]
    assert len(subscribed_topics) > 0

    # Verify that publication is allowed on chatter, but not subscription
    assert 'rt/chatter' in published_topics
    assert 'rt/chatter' not in subscribed_topics
