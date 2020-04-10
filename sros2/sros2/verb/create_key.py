# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

try:
    from argcomplete.completers import DirectoriesCompleter
except ImportError:
    def DirectoriesCompleter():
        return None

from sros2.api import _key
from sros2.verb import VerbExtension


class CreateKeyVerb(VerbExtension):
    """Create key."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument('ROOT', help='root path of keystore')
        arg.completer = DirectoriesCompleter()
        parser.add_argument('NAME', help='key name, aka ROS enclave name')

    def main(self, *, args):
        success = _key.create_key(args.ROOT, args.NAME)
        return 0 if success else 1
