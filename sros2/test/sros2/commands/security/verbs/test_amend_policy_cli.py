# Copyright 2019 Canonical Ltd
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
import tempfile

import pytest

import rclpy

from ament_index_python import get_package_prefix
from ros2cli import cli
from sros2.policy import load_policy
from std_msgs.msg import String
from test import get_policies_path


def test_ament_policy():
    # Create a test-specific context so that amend_policy can still init
    context = rclpy.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('node', namespace='ns', context=context)

    try:
        # Create a publisher and subscription
        node.create_publisher(String, 'denied_topic', 1)
        node.create_subscription(String, 'topic_sub', lambda msg: None, 1)

        # Generate the policy for the running node
        assert cli.main(
            argv=['security', 'amend_policy',
                  os.path.join(get_policies_path(), 'dummy_policy.xml'),
                  '-t 5']) is None
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)

    # # Load the policy and pull out the allowed publications and subscriptions
    # policy = load_policy(os.path.join(tmpdir, 'test-policy.xml'))
    # profile = policy.find(path='profiles/profile[@ns="/"][@node="test_node"]')
    # assert profile is not None
    # topics_publish_allowed = profile.find(path='topics[@publish="ALLOW"]')
    # assert topics_publish_allowed is not None
    # topics_subscribe_allowed = profile.find(path='topics[@subscribe="ALLOW"]')
    # assert topics_subscribe_allowed is not None
    #
    # # Verify that the allowed publications include topic_pub and not topic_sub
    # topics = topics_publish_allowed.findall('topic')
    # assert len([t for t in topics if t.text == 'topic_pub']) == 1
    # assert len([t for t in topics if t.text == 'topic_sub']) == 0
    #
    # # Verify that the allowed subscriptions include topic_sub and not topic_pub
    # topics = topics_subscribe_allowed.findall('topic')
    # assert len([t for t in topics if t.text == 'topic_sub']) == 1
    # assert len([t for t in topics if t.text == 'topic_pub']) == 0


def test_amend_policy_no_file(capsys):
    assert cli.main(argv=[
        'security', 'amend_policy', os.path.join('', 'bar.xml')]) == \
            'Package policy file not found'


def test_amend_policy_file_not_valid(capsys):
    assert cli.main(argv=[
        'security', 'amend_policy',
        os.path.join(get_policies_path(), 'invalid_policy.xml')]) == \
        'Package policy file not valid'
