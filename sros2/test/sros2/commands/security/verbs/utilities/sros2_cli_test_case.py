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

import argparse
import contextlib
import time
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import launch_testing_ros.tools

from ros2cli.node.strategy import NodeStrategy


MAX_DISCOVERY_DELAY = 4.0  # seconds


def generate_sros2_cli_test_description(
    fixture_actions, rmw_implementation, use_daemon
) -> LaunchDescription:
    additional_env = {'RMW_IMPLEMENTATION': rmw_implementation}
    if use_daemon:
        # Start daemon.
        fixture_actions = [ExecuteProcess(
            cmd=['ros2', 'daemon', 'start'],
            name='daemon-start',
            on_exit=fixture_actions,
            additional_env={k: str(v) for k, v in additional_env.items()}
        )]
    return LaunchDescription([
        # Always stop daemon to avoid cross-talk.
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=fixture_actions,
            additional_env={k: str(v) for k, v in additional_env.items()}
        ),
    ])


class SROS2CLITestCase(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output,
        rmw_implementation,
        use_daemon
    ):
        @contextlib.contextmanager
        def launch_sros2_command(self, arguments):
            cmd = ['ros2', 'security', *arguments]
            if not use_daemon:
                # Wait for direct node to discover fixture nodes.
                cmd.extend(['--no-daemon', '--spin-time', f'{MAX_DISCOVERY_DELAY}'])
            sros2_command_action = ExecuteProcess(
                cmd=cmd,
                additional_env={
                    'RMW_IMPLEMENTATION': rmw_implementation,
                    'PYTHONUNBUFFERED': '1'
                },
                name='ros2security-cli',
                output='screen'
            )
            with launch_testing.tools.launch_process(
                launch_service, sros2_command_action, proc_info, proc_output,
                output_filter=launch_testing_ros.tools.basic_output_filter(
                    # ignore launch_ros and ros2cli daemon nodes
                    filtered_patterns=['.*launch_ros*', '.*ros2cli.*'],
                    filtered_rmw_implementation=rmw_implementation
                )
            ) as sros2_command:
                yield sros2_command
        cls.launch_sros2_command = launch_sros2_command

        if use_daemon:
            def predicate(node, expected_nodes, expected_topics, expected_services):
                nodes = [
                    ns + '/' + name for name, ns in
                    node.get_node_names_and_namespaces()]
                if not all(name in nodes for name in expected_nodes):
                    return False
                if expected_topics:
                    topics = [name for name, _ in node.get_topic_names_and_types()]
                    if not all(name in topics for name in expected_topics):
                        return False
                if expected_services:
                    services = [name for name, _ in node.get_service_names_and_types()]
                    if not all(name in services for name in expected_services):
                        return False
                return True

            def wait_for(self, expected_nodes, expected_topics=None, expected_services=None):
                args = argparse.Namespace()
                args.no_daemon = not use_daemon
                args.spin_time = MAX_DISCOVERY_DELAY
                with NodeStrategy(args) as node:
                    start_time = time.time()
                    while time.time() - start_time < MAX_DISCOVERY_DELAY:
                        if predicate(node, expected_nodes, expected_topics, expected_services):
                            return True
                        time.sleep(0.1)  # this sleep time is arbitrary
                    return False
            cls.wait_for = wait_for
