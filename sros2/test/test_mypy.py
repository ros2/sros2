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

from ament_mypy.main import main
import pytest


@pytest.mark.mypy
@pytest.mark.linter
def test_mypy():
    distroname = ''
    if os.path.exists('/etc/os-release'):
        with open('/etc/os-release', 'r') as infp:
            for line in infp:
                if line.startswith('ID='):
                    split = line.strip().split('=')
                    if len(split) == 2:
                        distroname = split[1].lstrip('"').rstrip('"')
                    break

    if distroname == 'centos':
        # CentOS 7 uses a pip installation of importlib_resources to get access
        # to the importlib_resources API.  Due to a bug
        # (https://github.com/python/mypy/issues/1153), mypy cannot determine
        # the API in that case and so throws a warning.  If we see we are on
        # CentOS, skip the mypy checking; on all other platforms, run the
        # test as usual.
        pytest.skip('CentOS 7 does not support mypy checking of importlib properly')

    assert main(argv=[]) == 0, 'Found errors'
