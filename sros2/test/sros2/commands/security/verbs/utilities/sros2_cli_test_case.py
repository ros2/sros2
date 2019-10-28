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

import contextlib
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import TimerAction

import launch_testing
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools
import launch_testing_ros.tools


MAX_DISCOVERY_DELAY = 2.0  # seconds


def generate_sros2_cli_test_description(
    fixture_actions, rmw_implementation, use_daemon
):
    additional_env = {'RMW_IMPLEMENTATION': rmw_implementation}
    if use_daemon:
        # Start daemon.
        fixture_actions = [ExecuteProcess(
            cmd=['ros2', 'daemon', 'start'],
            name='daemon-start',
            # Wait for daemon to discover fixture nodes.
            on_exit=[TimerAction(
                period=MAX_DISCOVERY_DELAY,
                actions=fixture_actions
            )],
            additional_env=additional_env
        )]
    return LaunchDescription([
        # Always stop daemon to avoid cross-talk.
        ExecuteProcess(
            cmd=['ros2', 'daemon', 'stop'],
            name='daemon-stop',
            on_exit=fixture_actions,
            additional_env=additional_env
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
