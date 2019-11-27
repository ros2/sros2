# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from ros2cli import cli


def test_generate_policy_no_nodes(capsys):
    with tempfile.TemporaryDirectory() as tmpdir:
        assert cli.main(argv=[
            'security', 'generate_policy', os.path.join(tmpdir, 'test-policy.xml')]) != 0
        stderr = capsys.readouterr().err.strip()
        assert stderr == 'No nodes detected in the ROS graph. No policy file was generated.'
