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

from ament_mypy.main import main
import pytest


@pytest.mark.mypy
@pytest.mark.linter
def test_mypy():
    try:
        import importlib.resources as _  # noqa: F401
    except ModuleNotFoundError:
        # The importlib_resources package is a backport of the importlib.resources module
        # from Python 3.9. The 'policy' module of this project first tries to import from
        # importlib.resources, then falls back to the backport package.
        # There is a bug in mypy that manifests when this try/except import pattern is
        # used: https://github.com/python/mypy/issues/1153
        pytest.skip('This platform does not support mypy checking of importlib properly')
    assert main(argv=[]) == 0, 'Found errors'
