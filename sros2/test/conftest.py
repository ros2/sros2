# Copyright 2019 Apex.AI, Inc.
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

import pytest

# Disable lxml2 lookup of resources on the internet by configuring it to use a proxy
# that does not exist
os.environ['HTTP_PROXY'] = 'http://example.com'


@pytest.fixture(scope='session')
def test_policy_dir():
    return pathlib.Path(__file__).parent / 'policies'
