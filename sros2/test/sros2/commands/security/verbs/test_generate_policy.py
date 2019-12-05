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

import contextlib
import os
import sys
import tempfile
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import launch_testing_ros.tools

import pytest

from rmw_implementation import get_available_rmw_implementations
from sros2.policy import load_policy


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation', get_available_rmw_implementations())
def generate_test_description(rmw_implementation):
    additional_env = {'RMW_IMPLEMENTATION': rmw_implementation}
    path_to_pub_sub_node_script = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'pub_sub_node.py'
    )
    path_to_client_srv_node_script = os.path.join(
        os.path.dirname(__file__), 'fixtures', 'client_service_node.py'
    )
    pub_sub_node = Node(
        node_executable=sys.executable,
        arguments=[path_to_pub_sub_node_script],
        node_name='test_generate_policy_topics_node',
        additional_env=additional_env
    )
    client_srv_node = Node(
        node_executable=sys.executable,
        arguments=[path_to_client_srv_node_script],
        node_name='test_generate_policy_services_node',
        additional_env=additional_env
    )
    return LaunchDescription([
        # Always restart daemon to isolate tests.
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=[
                ExecuteProcess(
                    cmd=['ros2', 'daemon', 'start'],
                    name='daemon-start',
                    on_exit=[
                        pub_sub_node, client_srv_node, ReadyToTest()
                    ],
                    additional_env=additional_env
                )
            ]
        ),
    ])


class TestSROS2GeneratePolicyVerb(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation
    ):
        @contextlib.contextmanager
        def launch_gen_policy_command(self, arguments):
            gen_policy_command_action = ExecuteProcess(
                cmd=['ros2', 'security', 'generate_policy', *arguments],
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2security-gen-policy-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, gen_policy_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    # ignore launch_ros and ros2cli daemon nodes
                    filtered_patterns=['.*launch_ros*', '.*ros2cli.*'],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as gen_policy_command:
                yield gen_policy_command
        cls.launch_gen_policy_command = launch_gen_policy_command

    @launch_testing.markers.retry_on_failure(times=3)
    def test_generate_policy_topics(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            with self.launch_gen_policy_command(
                arguments=[os.path.join(tmpdir, 'test-policy.xml')]
            ) as gen_command:
                assert gen_command.wait_for_shutdown(timeout=10)
            assert gen_command.exit_code == launch_testing.asserts.EXIT_OK

            # Load the policy and pull out the allowed publications and subscriptions
            policy = load_policy(os.path.join(tmpdir, 'test-policy.xml'))
            profile = policy.find(
                path='profiles/profile[@ns="/"][@node="test_generate_policy_topics_node"]')
            assert profile is not None
            topics_publish_allowed = profile.find(path='topics[@publish="ALLOW"]')
            assert topics_publish_allowed is not None
            topics_subscribe_allowed = profile.find(path='topics[@subscribe="ALLOW"]')
            assert topics_subscribe_allowed is not None

            # Verify that the allowed publications include topic_pub and not topic_sub
            topics = topics_publish_allowed.findall('topic')
            assert len([t for t in topics if t.text == 'test_generate_policy_topics_pub']) == 1
            assert len([t for t in topics if t.text == 'test_generate_policy_topics_sub']) == 0

            # Verify that the allowed subscriptions include topic_sub and not topic_pub
            topics = topics_subscribe_allowed.findall('topic')
            assert len([t for t in topics if t.text == 'test_generate_policy_topics_sub']) == 1
            assert len([t for t in topics if t.text == 'test_generate_policy_topics_pub']) == 0

    @launch_testing.markers.retry_on_failure(times=3)
    def test_generate_policy_services(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            with self.launch_gen_policy_command(
                arguments=[os.path.join(tmpdir, 'test-policy.xml')]
            ) as gen_command:
                assert gen_command.wait_for_shutdown(timeout=10)
            assert gen_command.exit_code == launch_testing.asserts.EXIT_OK

            # Load the policy and pull out allowed replies and requests
            policy = load_policy(os.path.join(tmpdir, 'test-policy.xml'))
            profile = policy.find(
                path='profiles/profile[@ns="/"][@node="test_generate_policy_services_node"]')
            assert profile is not None
            service_reply_allowed = profile.find(path='services[@reply="ALLOW"]')
            assert service_reply_allowed is not None
            service_request_allowed = profile.find(path='services[@request="ALLOW"]')
            assert service_request_allowed is not None

            # Verify that the allowed replies include service_server and not service_client
            services = service_reply_allowed.findall('service')
            assert len(
                [s for s in services if s.text == 'test_generate_policy_services_server']) == 1
            assert len(
                [s for s in services if s.text == 'test_generate_policy_services_client']) == 0

            # Verify that the allowed requests include service_client and not service_server
            services = service_request_allowed.findall('service')
            assert len(
                [s for s in services if s.text == 'test_generate_policy_services_client']) == 1
            assert len(
                [s for s in services if s.text == 'test_generate_policy_services_server']) == 0

    @launch_testing.markers.retry_on_failure(times=2)
    def test_generate_policy_no_policy_file(self):
        with self.launch_gen_policy_command(arguments=[]) as gen_command:
            assert gen_command.wait_for_shutdown(timeout=10)
        assert gen_command.exit_code != launch_testing.asserts.EXIT_OK
        assert launch_testing.tools.expect_output(
            expected_lines=[
                'usage: ros2 security generate_policy [-h] POLICY_FILE_PATH',
                'ros2 security generate_policy: error: the following'
                ' arguments are required: POLICY_FILE_PATH'
            ],
            text=gen_command.output,
            strict=False
        )
