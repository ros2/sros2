# Copyright 2019 Amazon.com, Inc or its affiliates. All Rights Reserved.
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

from contextlib import ExitStack
import os
import tempfile

from ros2cli import cli
from sros2.api import create_key
from sros2.api import create_keystore
# from sros2.api import generate_policy


def test_create_permission(capsys):
    node_name = '/test_node'
    with ExitStack() as estack:
        tmpdir = estack.enter_context(tempfile.TemporaryDirectory())
        keystore_dir = estack.enter_context(tempfile.TemporaryDirectory())
        with capsys.disabled():
            assert create_keystore(keystore_dir)
            assert create_key(keystore_dir, node_name)
            # assert generate_policy(tmpdir)

        assert cli.main(argv=[
            'security', 'create_permission', keystore_dir, node_name, tmpdir
        ]) == 0
        assert capsys.readouterr().out.strip() == 'wfpwfpyul'
