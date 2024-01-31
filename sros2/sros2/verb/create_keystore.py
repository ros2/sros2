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

import pathlib
import sys

from argcomplete.completers import DirectoriesCompleter

import sros2.errors
import sros2.keystore
from sros2.verb import VerbExtension


class CreateKeystoreVerb(VerbExtension):
    """Create keystore."""

    def add_arguments(self, parser, cli_name) -> None:
        arg = parser.add_argument('ROOT', type=pathlib.Path, help='root path of keystore')
        arg.completer = DirectoriesCompleter()

    def main(self, *, args) -> int:
        try:
            sros2.keystore.create_keystore(args.ROOT)
        except sros2.errors.SROS2Error as e:
            print(f'Unable to create keystore: {str(e)}', file=sys.stderr)
            return 1
        return 0
