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
import pathlib
import tempfile

from ros2cli import cli

import sros2.keystore


def test_list_enclaves(capsys):
    enclave_names = ['/test_enclave', '/test/nested_enclave', '/sky/is/the/limit']
    with tempfile.TemporaryDirectory() as keystore_dir:
        keystore_dir = pathlib.Path(keystore_dir)

        with capsys.disabled():
            # First, create the keystore
            sros2.keystore.create_keystore(keystore_dir)
            assert keystore_dir.is_dir()

            # Now using that keystore, create an enclave
            for enclave_name in enclave_names:
                sros2.keystore.create_enclave(keystore_dir, enclave_name)

        # Now verify that the enclave we just created is included in the list
        assert cli.main(argv=['security', 'list_enclaves', str(keystore_dir)]) == 0
        assert capsys.readouterr().out.strip() == '\n'.join(sorted(enclave_names))


def test_list_enclaves_no_keys(capsys):
    with tempfile.TemporaryDirectory() as keystore_dir:
        keystore_dir = pathlib.Path(keystore_dir)

        with capsys.disabled():
            # First, create the keystore
            sros2.keystore.create_keystore(keystore_dir)
            assert keystore_dir.is_dir()

        # Now verify that empty keystore we just created contains no enclaves
        assert cli.main(argv=['security', 'list_enclaves', str(keystore_dir)]) == 0
        assert len(capsys.readouterr().out.strip()) == 0


def test_list_enclaves_uninitialized_keystore(capsys):
    with tempfile.TemporaryDirectory() as keystore_dir:
        # Verify that list_enclaves properly handles an uninitialized keystore
        assert cli.main(argv=['security', 'list_enclaves', keystore_dir]) == 1
        assert (capsys.readouterr().err.strip() ==
                f"Unable to list enclaves: '{keystore_dir}' is not a valid keystore")


def test_list_enclaves_no_keystore(capsys):
    # Verify that list_enclaves properly handles a non-existent keystore
    keystore = os.path.join(tempfile.gettempdir(), 'non-existent')
    assert cli.main(argv=['security', 'list_enclaves', keystore]) == 1
    assert (capsys.readouterr().err.strip() ==
            f"Unable to list enclaves: '{keystore}' is not a valid keystore")
