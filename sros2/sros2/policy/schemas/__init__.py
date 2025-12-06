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

"""
Workaround to make importlib.resources path lookup work.

importlib.resources in Python 3.8 and earlier doesn't support looking up paths
in subdirectories.  However, it does support looking up paths in python modules,
so we add this __init__.py to make this subdirectoy a module.  This workaround
can be removed when all of the target platforms support Python 3.9 and later;
see https://gitlab.com/python-devs/importlib_resources/issues/58 for details.
"""
