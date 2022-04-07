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

import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools

import pytest

from rclpy.utilities import get_available_rmw_implementations

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utilities.sros2_cli_test_case import generate_sros2_cli_test_description  # noqa: E402
from utilities.sros2_cli_test_case import SROS2CLITestCase  # noqa: E402


@pytest.mark.rostest
@launch_testing.parametrize('rmw_implementation,use_daemon', itertools.product(
    get_available_rmw_implementations(), (True, False)
))
@launch_testing.markers.keep_alive
def generate_test_description(rmw_implementation, use_daemon):
    return generate_sros2_cli_test_description(
        fixture_actions=[ReadyToTest()],
        rmw_implementation=rmw_implementation,
        use_daemon=use_daemon
    ), locals()


GENERATE_POLICY_TIMEOUT = 10 if os.name != 'nt' else 30  # seconds


class TestSROS2GeneratePolicyVerbWithNoNodes(SROS2CLITestCase):

    def test_generate_policy_no_nodes(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            test_policy = pathlib.Path(tmpdir).joinpath('test-policy.xml')
            with self.launch_sros2_command(
                arguments=['generate_policy', str(test_policy)]
            ) as gen_command:
                assert gen_command.wait_for_shutdown(timeout=GENERATE_POLICY_TIMEOUT)
            assert gen_command.exit_code != launch_testing.asserts.EXIT_OK
            assert 'No nodes detected in the ROS graph. No policy file was generated.' \
                in gen_command.output

    def test_generate_policy_no_policy_file(self):
        with self.launch_sros2_command(arguments=['generate_policy']) as gen_command:
            assert gen_command.wait_for_shutdown(timeout=GENERATE_POLICY_TIMEOUT)
        assert gen_command.exit_code != launch_testing.asserts.EXIT_OK
        assert 'following arguments are required: POLICY_FILE_PATH' in gen_command.output
