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
import textwrap

import pytest

import rclpy
from ros2cli import cli
from sros2.policy import load_policy
from std_msgs.msg import String


def test_generate_policy():
    with tempfile.TemporaryDirectory() as tmpdir:
        # Create a test-specific context so that generate_policy can still init
        context = rclpy.Context()
        rclpy.init(context=context)
        node = rclpy.create_node('test_node', context=context)

        try:
            # Create a publisher and subscription
            node.create_publisher(String, 'topic_pub', 1)
            node.create_subscription(String, '/topic_sub', lambda msg: None, 1)
            node.create_publisher(String, 'topic/both', 1)
            node.create_subscription(String, 'topic/both', lambda msg: None, 1)

            # Generate the policy for the running node
            assert cli.main(
                argv=['security', 'generate_policy', os.path.join(tmpdir, 'test-policy.xml')]) == 0
        finally:
            node.destroy_node()
            rclpy.shutdown(context=context)

        # Load the policy and pull out the allowed publications and subscriptions
        policy = load_policy(os.path.join(tmpdir, 'test-policy.xml'))
        profile = policy.find(path='profiles/profile[@ns="/"][@node="test_node"]')
        assert profile is not None
        topics_publish_allowed = profile.find(path='topics[@publish="ALLOW"]')
        assert topics_publish_allowed is not None
        topics_subscribe_allowed = profile.find(path='topics[@subscribe="ALLOW"]')
        assert topics_subscribe_allowed is not None

        # Verify that the allowed publications include topic_pub and topic_both, but not
        # topic_sub
        topics = topics_publish_allowed.findall('topic')
        assert len([t for t in topics if t.text == 'topic_pub']) == 1
        assert len([t for t in topics if t.text == 'topic/both']) == 1
        assert len([t for t in topics if t.text == 'topic_sub']) == 0

        # Verify that the allowed subscriptions include topic_sub and topic_both, but not
        # topic_pub
        topics = topics_subscribe_allowed.findall('topic')
        assert len([t for t in topics if t.text == 'topic_sub']) == 1
        assert len([t for t in topics if t.text == 'topic/both']) == 1
        assert len([t for t in topics if t.text == 'topic_pub']) == 0

        # Couple last sanity checks to ensure various expression formats are supported
        services_reply_allowed = profile.find(path='services[@reply="ALLOW"]')
        assert services_reply_allowed is not None

        services = services_reply_allowed.findall('service')
        assert len([s for s in services if s.text == '~describe_parameters']) == 1


def test_generate_policy_append():
    with tempfile.TemporaryDirectory() as tmpdir:
        # Create a test-specific context so that generate_policy can still init
        context = rclpy.Context()
        rclpy.init(context=context)
        node = rclpy.create_node('test_node', context=context)

        # Create an initial policy
        with open(os.path.join(tmpdir, 'test-policy.xml'), 'w') as f:
            f.write(textwrap.dedent("""\
                <?xml version="1.0" encoding="UTF-8"?>
                <policy version="0.1.0">
                    <profiles>
                        <profile ns="/" node="test_node">
                            <topics publish="ALLOW" >
                                <topic>topic_pub</topic>
                            </topics>
                        </profile>
                    </profiles>
                </policy>
            """))

        try:
            # Create a subscription
            node.create_subscription(String, '/topic_sub', lambda msg: None, 1)

            # Generate the policy for the running node, based on the existing policy
            assert cli.main(
                argv=['security', 'generate_policy', os.path.join(tmpdir, 'test-policy.xml')]) == 0
        finally:
            node.destroy_node()
            rclpy.shutdown(context=context)

        # Load the policy and pull out the allowed publications and subscriptions
        policy = load_policy(os.path.join(tmpdir, 'test-policy.xml'))
        profile = policy.find(path='profiles/profile[@ns="/"][@node="test_node"]')
        assert profile is not None
        topics_publish_allowed = profile.find(path='topics[@publish="ALLOW"]')
        assert topics_publish_allowed is not None
        topics_subscribe_allowed = profile.find(path='topics[@subscribe="ALLOW"]')
        assert topics_subscribe_allowed is not None

        # Verify that the allowed publications include topic_pub, but not topic_sub
        topics = topics_publish_allowed.findall('topic')
        assert len([t for t in topics if t.text == 'topic_pub']) == 1
        assert len([t for t in topics if t.text == 'topic_sub']) == 0

        # Verify that the allowed subscriptions include topic_sub, but not topic_pub
        topics = topics_subscribe_allowed.findall('topic')
        assert len([t for t in topics if t.text == 'topic_sub']) == 1
        assert len([t for t in topics if t.text == 'topic_pub']) == 0


def test_generate_policy_no_nodes(capsys):
    with tempfile.TemporaryDirectory() as tmpdir:
        assert cli.main(argv=[
            'security', 'generate_policy', os.path.join(tmpdir, 'test-policy.xml')]) != 0
        stderr = capsys.readouterr().err.strip()
        assert stderr == 'No nodes detected in the ROS graph. No policy file was generated.'


def test_generate_policy_no_policy_file(capsys):
    with pytest.raises(SystemExit) as e:
        cli.main(argv=['security', 'generate_policy'])
        assert e.value.code != 0
    stderr = capsys.readouterr().err.strip()
    assert 'following arguments are required: POLICY_FILE_PATH' in stderr
