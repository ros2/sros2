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

from pathlib import Path

import pytest

from ros2cli import cli

from sros2.errors import (
    InvalidEnclaveNameError,
    InvalidKeystoreError,
)
from sros2.keystore import _enclave


def test_is_key_name_valid():
    # Valid cases
    assert _enclave._is_enclave_name_valid('/foo')
    assert _enclave._is_enclave_name_valid('/foo/bar')
    assert _enclave._is_enclave_name_valid('/foo/bar123/_/baz_')

    # Invalid cases
    assert not _enclave._is_enclave_name_valid('')
    assert not _enclave._is_enclave_name_valid(' ')
    assert not _enclave._is_enclave_name_valid('//')
    assert not _enclave._is_enclave_name_valid('foo')
    assert not _enclave._is_enclave_name_valid('foo/bar')
    assert not _enclave._is_enclave_name_valid('/42foo')
    assert not _enclave._is_enclave_name_valid('/foo/42bar')


@pytest.fixture()
def keystore_dir(tmp_path_factory) -> Path:
    keystore_dir = tmp_path_factory.mktemp('keystore')

    # Create the keystore
    assert cli.main(argv=['security', 'create_keystore', str(keystore_dir)]) == 0

    # Return path to keystore directory
    return keystore_dir


def test_create_enclave_invalid_arguments(keystore_dir):
    with pytest.raises(InvalidKeystoreError):
        _enclave.create_enclave(Path('foo/bar'), '/baz/foobar')
    with pytest.raises(InvalidKeystoreError):
        _enclave.create_enclave(Path('foo/bar'), 'baz/foobar')
    with pytest.raises(InvalidEnclaveNameError):
        _enclave.create_enclave(keystore_dir, 'baz/foobar')
