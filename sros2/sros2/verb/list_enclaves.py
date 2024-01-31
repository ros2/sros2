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
import warnings

from argcomplete.completers import DirectoriesCompleter

import sros2.keystore
from sros2.verb import VerbExtension


class ListEnclavesVerb(VerbExtension):
    """List enclaves in keystore."""

    def add_arguments(self, parser, cli_name) -> None:
        arg = parser.add_argument('ROOT', type=pathlib.Path, help='root path of keystore')
        arg.completer = DirectoriesCompleter()

    def main(self, *, args) -> int:
        try:
            enclaves = sros2.keystore.get_enclaves(args.ROOT)
            if enclaves:
                # Print each enclave on its own line
                for enclave in sorted(enclaves):
                    print(enclave)
            return 0
        except sros2.errors.SROS2Error as e:
            print(f'Unable to list enclaves: {str(e)}', file=sys.stderr)
        return 1


class ListKeysVerb(ListEnclavesVerb):
    """DEPRECATED: List enclaves in keystore. Use list_enclaves instead."""

    def main(self, *, args) -> int:
        warnings.warn(
            'list_keys is deprecated and will be removed in a future release. Use list_enclaves '
            'instead.', FutureWarning)
        return super().main(args=args)
