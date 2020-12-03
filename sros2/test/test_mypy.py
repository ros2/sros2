# Copyright 2019 Canonical Ltd
# Copyright 2020 Mikael Arguedas
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
from rospkg.os_detect import read_os_release


@pytest.mark.mypy
@pytest.mark.linter
def test_mypy():
    release_info = read_os_release()
    if isinstance(release_info, dict) and all(key in release_info for key in ['ID', 'VERSION_ID']):
        if release_info['ID'] == 'centos' and release_info['VERSION_ID'] == '7':
            # CentOS 7 uses a pip installation of importlib_resources to get access
            # to the importlib_resources API.  Due to a bug
            # (https://github.com/python/mypy/issues/1153), mypy cannot determine
            # the API in that case and so throws a warning.  If we see we are on
            # CentOS, skip the mypy checking; on all other platforms, run the
            # test as usual.
            pytest.skip('CentOS 7 does not support mypy checking of importlib properly')
    assert main(argv=[]) == 0, 'Found errors'
