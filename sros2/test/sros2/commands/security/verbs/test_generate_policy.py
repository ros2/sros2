# Copyright 2019 Canonical Ltd
# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import itertools
import os
import pathlib
import sys
import tempfile
import unittest

from launch_ros.actions import Node

import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools

import pytest

from rclpy.expand_topic_name import expand_topic_name
from rclpy.utilities import get_available_rmw_implementations
from sros2.policy import load_policy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utilities.sros2_cli_test_case import generate_sros2_cli_test_description  # noqa: E402
from utilities.sros2_cli_test_case import SROS2CLITestCase  # noqa: E402


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation,use_daemon', itertools.product(
    get_available_rmw_implementations(), (True, False)
))
def generate_test_description(rmw_implementation: str, use_daemon: bool):
    if 'connext' in rmw_implementation and not use_daemon:
        raise unittest.SkipTest(
            f'Using {rmw_implementation} w/o a daemon makes tests flaky'
        )

    additional_env = {'RMW_IMPLEMENTATION': rmw_implementation}
    path_to_pub_sub_node_script = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'pub_sub_node.py'
    )
    pub_sub_node_enclave = '/foo/bar'
    pub_sub_node_namespace = '/'
    pub_sub_node_name = 'test_generate_policy_topics_node'
    pub_sub_node = Node(
        executable=sys.executable,
        arguments=[
            path_to_pub_sub_node_script,
            '--ros-args',
            '-e', pub_sub_node_enclave
        ],
        name=pub_sub_node_name,
        namespace=pub_sub_node_namespace,
        additional_env=additional_env
    )
    path_to_client_srv_node_script = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'client_service_node.py'
    )
    client_srv_node_enclave = '/foo'
    client_srv_node_namespace = '/node_ns'
    client_srv_node_name = 'test_generate_policy_services_node'
    client_srv_node = Node(
        executable=sys.executable,
        arguments=[
            path_to_client_srv_node_script,
            '--ros-args',
            '-e', client_srv_node_enclave
        ],
        name=client_srv_node_name,
        namespace=client_srv_node_namespace,
        additional_env=additional_env
    )
    return generate_sros2_cli_test_description(
        fixture_actions=[
            pub_sub_node,
            client_srv_node,
            ReadyToTest()
        ],
        rmw_implementation=rmw_implementation,
        use_daemon=use_daemon
    ), locals()


GENERATE_POLICY_TIMEOUT = 10 if os.name != 'nt' else 30  # seconds


class TestSROS2GeneratePolicyVerb(SROS2CLITestCase):

    def test_generate_policy_topics(
        self,
        pub_sub_node_name,
        pub_sub_node_namespace,
        pub_sub_node_enclave,
        use_daemon
    ):
        if use_daemon:
            assert self.wait_for(
                expected_nodes=[pub_sub_node_namespace + '/' + pub_sub_node_name],
                expected_topics=[
                    expand_topic_name('~/pub', pub_sub_node_name, pub_sub_node_namespace),
                    expand_topic_name('~/sub', pub_sub_node_name, pub_sub_node_namespace),
                ]
            )

        with tempfile.TemporaryDirectory() as tmpdir:
            test_policy = pathlib.Path(tmpdir).joinpath('test-policy.xml')
            with self.launch_sros2_command(
                arguments=['generate_policy', str(test_policy)]
            ) as gen_command:
                assert gen_command.wait_for_shutdown(timeout=GENERATE_POLICY_TIMEOUT)
            assert gen_command.exit_code == launch_testing.asserts.EXIT_OK

            # Load the policy and pull out the allowed publications and subscriptions
            policy = load_policy(test_policy)
            profile = policy.find(
                path=f'enclaves/enclave[@path="{pub_sub_node_enclave}"]'
                     + f'/profiles/profile[@ns="{pub_sub_node_namespace}"]'
                     + f'[@node="{pub_sub_node_name}"]'
            )
            assert profile is not None
            topics_publish_allowed = profile.find(path='topics[@publish="ALLOW"]')
            assert topics_publish_allowed is not None
            topics_subscribe_allowed = profile.find(path='topics[@subscribe="ALLOW"]')
            assert topics_subscribe_allowed is not None

            # Verify that the allowed publications include topic_pub and not topic_sub
            topics = topics_publish_allowed.findall('topic')
            assert len([t for t in topics if t.text == '~/pub']) == 1
            assert len([t for t in topics if t.text == '~/sub']) == 0

            # Verify that the allowed subscriptions include topic_sub and not topic_pub
            topics = topics_subscribe_allowed.findall('topic')
            assert len([t for t in topics if t.text == '~/sub']) == 1
            assert len([t for t in topics if t.text == '~/pub']) == 0

    def test_generate_policy_services(
        self,
        client_srv_node_name,
        client_srv_node_namespace,
        client_srv_node_enclave,
        use_daemon
    ):
        if use_daemon:
            assert self.wait_for(
                expected_nodes=[client_srv_node_namespace + '/' + client_srv_node_name],
                expected_services=[
                    expand_topic_name('~/client', client_srv_node_name, client_srv_node_namespace),
                    expand_topic_name('~/server', client_srv_node_name, client_srv_node_namespace),
                ]
            )

        with tempfile.TemporaryDirectory() as tmpdir:
            test_policy = pathlib.Path(tmpdir).joinpath('test-policy.xml')
            with self.launch_sros2_command(
                arguments=['generate_policy', str(test_policy)]
            ) as gen_command:
                assert gen_command.wait_for_shutdown(timeout=GENERATE_POLICY_TIMEOUT)
            assert gen_command.exit_code == launch_testing.asserts.EXIT_OK

            # Load the policy and pull out allowed replies and requests
            policy = load_policy(test_policy)
            profile = policy.find(
                path=f'enclaves/enclave[@path="{client_srv_node_enclave}"]'
                     + f'/profiles/profile[@ns="{client_srv_node_namespace}"]'
                     + f'[@node="{client_srv_node_name}"]'
            )
            assert profile is not None
            service_reply_allowed = profile.find(path='services[@reply="ALLOW"]')
            assert service_reply_allowed is not None
            service_request_allowed = profile.find(path='services[@request="ALLOW"]')
            assert service_request_allowed is not None

            # Verify that the allowed replies include service_server and not service_client
            services = service_reply_allowed.findall('service')
            assert len(
                [s for s in services if s.text == '~/server']) == 1
            assert len(
                [s for s in services if s.text == '~/client']) == 0

            # Verify that the allowed requests include service_client and not service_server
            services = service_request_allowed.findall('service')
            assert len(
                [s for s in services if s.text == '~/client']) == 1
            assert len(
                [s for s in services if s.text == '~/server']) == 0
