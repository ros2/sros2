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
from ros2cli import cli
from sros2.policy import load_policy
from test_msgs.msg import Strings
from test_msgs.srv import Empty


def test_generate_policy_topics():
    with tempfile.TemporaryDirectory() as tmpdir:
        test_enclave = '/foo/bar'
        test_node_namespace = '/'
        test_name = 'test_generate_policy_topics'
        test_node_name = f'{test_name}_node'
        # Create a test-specific context so that generate_policy can still init
        context = rclpy.Context()
        rclpy.init(context=context, args=['--ros-args', '-e', test_enclave])
        node = rclpy.create_node(test_node_name, context=context)

        try:
            # Create a publisher and subscription
            node.create_publisher(Strings, f'{test_name}_pub', 1)
            node.create_subscription(Strings, f'{test_name}_sub', lambda msg: None, 1)

            # Generate the policy for the running node
            assert cli.main(
                argv=['security', 'generate_policy', os.path.join(tmpdir, 'test-policy.xml')]) == 0
        finally:
            node.destroy_node()
            rclpy.shutdown(context=context)

        # Load the policy and pull out the allowed publications and subscriptions
        policy = load_policy(os.path.join(tmpdir, 'test-policy.xml'))
        profile = policy.find(
            path=f'enclaves/enclave[@path="{test_enclave}"]'
                 + f'/profiles/profile[@ns="{test_node_namespace}"]'
                 + f'[@node="{test_node_name}"]'
        )
        assert profile is not None
        topics_publish_allowed = profile.find(path='topics[@publish="ALLOW"]')
        assert topics_publish_allowed is not None
        topics_subscribe_allowed = profile.find(path='topics[@subscribe="ALLOW"]')
        assert topics_subscribe_allowed is not None

        # Verify that the allowed publications include topic_pub and not topic_sub
        topics = topics_publish_allowed.findall('topic')
        assert len([t for t in topics if t.text == f'{test_name}_pub']) == 1
        assert len([t for t in topics if t.text == f'{test_name}_sub']) == 0

        # Verify that the allowed subscriptions include topic_sub and not topic_pub
        topics = topics_subscribe_allowed.findall('topic')
        assert len([t for t in topics if t.text == f'{test_name}_sub']) == 1
        assert len([t for t in topics if t.text == f'{test_name}_pub']) == 0


def test_generate_policy_services():
    with tempfile.TemporaryDirectory() as tmpdir:
        # Create a test-specific context so that generate_policy can still init
        context = rclpy.Context()
        test_enclave = '/foo'
        test_node_namespace = '/node_ns'
        test_name = 'test_generate_policy_services'
        test_node_name = test_name + '_node'
        rclpy.init(context=context, args=['--ros-args', '-e', test_enclave])
        node = rclpy.create_node(
                test_node_name,
                namespace=test_node_namespace,
                context=context
        )

        try:
            # Create a server and client
            node.create_client(Empty, f'{test_name}_client')
            node.create_service(Empty, f'{test_name}_server', lambda request,
                                response: response)

            # Generate the policy for the running node
            assert cli.main(
                argv=['security', 'generate_policy', os.path.join(tmpdir, 'test-policy.xml')]) == 0
        finally:
            node.destroy_node()
            rclpy.shutdown(context=context)

        # Load the policy and pull out allowed replies and requests
        policy = load_policy(os.path.join(tmpdir, 'test-policy.xml'))
        profile = policy.find(
            path=f'enclaves/enclave[@path="{test_enclave}"]'
                 + f'/profiles/profile[@ns="{test_node_namespace}"]'
                 + f'[@node="{test_node_name}"]'
        )
        assert profile is not None
        service_reply_allowed = profile.find(path='services[@reply="ALLOW"]')
        assert service_reply_allowed is not None
        service_request_allowed = profile.find(path='services[@request="ALLOW"]')
        assert service_request_allowed is not None

        # Verify that the allowed replies include service_server and not service_client
        services = service_reply_allowed.findall('service')
        assert len([s for s in services if s.text == f'{test_name}_server']) == 1
        assert len([s for s in services if s.text == f'{test_name}_client']) == 0

        # Verify that the allowed requests include service_client and not service_server
        services = service_request_allowed.findall('service')
        assert len([s for s in services if s.text == f'{test_name}_client']) == 1
        assert len([s for s in services if s.text == f'{test_name}_server']) == 0


# TODO(jacobperron): On Windows, this test is flakey due to nodes left-over from tests in
#                    other packages.
#                    See: https://github.com/ros2/sros2/issues/143
@pytest.mark.skipif(
    'nt' == os.name, reason='flakey due to nodes left-over from tests in other packages')
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
