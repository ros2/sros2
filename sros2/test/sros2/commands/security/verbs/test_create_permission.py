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

from ros2cli import cli
from sros2.api import _key, _keystore


# This fixture will run once for the entire module (as opposed to once per test)
@pytest.fixture(scope='module')
def security_context_dir(tmpdir_factory, test_policy_dir) -> pathlib.Path:
    keystore_dir = pathlib.Path(str(tmpdir_factory.mktemp('keystore')))

    # First, create the keystore as well as a keypair for the talker
    assert _keystore.create_keystore(keystore_dir)
    assert _key.create_key(keystore_dir, '/talker_listener/talker')

    security_files_dir = keystore_dir / 'contexts' / 'talker_listener' / 'talker'
    assert security_files_dir.is_dir()

    # Now using that keystore, create a permissions file using the sample policy
    policy_file_path = test_policy_dir / 'sample.policy.xml'
    assert cli.main(
        argv=[
            'security', 'create_permission', str(keystore_dir), '/talker_listener/talker',
            str(policy_file_path)]) == 0

    # Return path to directory containing the identity's files
    return security_files_dir


def test_create_permission(security_context_dir):
    assert security_context_dir.joinpath('permissions.xml').is_file()
    assert security_context_dir.joinpath('permissions.p7s').is_file()

    # Give the generated permissions XML a smoke test
    tree = lxml.etree.parse(str(security_context_dir.joinpath('permissions.xml')))

    dds = tree.getroot()
    assert dds.tag == 'dds'

    permissions = list(dds.iterchildren(tag='permissions'))
    assert len(permissions) == 1

    grants = list(permissions[0].iterchildren(tag='grant'))
    assert len(grants) == 1
    assert grants[0].get('name') == '/talker_listener/talker'

    allow_rules = list(grants[0].iterchildren(tag='allow_rule'))
    assert len(allow_rules) == 1

    publish_rules = list(allow_rules[0].iterchildren(tag='publish'))
    assert len(publish_rules) == 1

    subscribe_rules = list(allow_rules[0].iterchildren(tag='subscribe'))
    assert len(subscribe_rules) == 1

    published_topics_set = list(publish_rules[0].iterchildren(tag='topics'))
    assert len(published_topics_set) == 1
    published_topics = [c.text for c in published_topics_set[0].iterchildren(tag='topic')]
    assert len(published_topics) == 15

    subscribed_topics_set = list(subscribe_rules[0].iterchildren(tag='topics'))
    assert len(subscribed_topics_set) == 1
    subscribed_topics = list(subscribed_topics_set[0].iterchildren(tag='topic'))
    assert len(subscribed_topics) == 14

    # Verify that publication is allowed on chatter, but not subscription
    assert 'rt/chatter' in published_topics
    assert 'rt/chatter' not in subscribed_topics
