# Copyright 2020 Canonical Ltd
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

import pathlib

import pytest

from sros2 import _utilities
import sros2.errors


def test_get_keystore_path_from_env(monkeypatch):
    monkeypatch.setenv(_utilities._KEYSTORE_DIR_ENV, '/keystore/path')
    assert _utilities.get_keystore_path_from_env() == pathlib.Path('/keystore/path')


def test_get_keystore_path_from_env_error(monkeypatch):
    monkeypatch.delenv(_utilities._KEYSTORE_DIR_ENV, raising=False)

    with pytest.raises(sros2.errors.InvalidKeystoreEnvironmentError) as e:
        _utilities.get_keystore_path_from_env()

    assert e.value.variable_name == _utilities._KEYSTORE_DIR_ENV
