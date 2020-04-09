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

try:
    from argcomplete.completers import DirectoriesCompleter
except ImportError:
    def DirectoriesCompleter():
        return None
try:
    from argcomplete.completers import FilesCompleter
except ImportError:
    def FilesCompleter(*, allowednames, directories):
        return None

from sros2.api import _artifact_generation
from sros2.verb import VerbExtension


class GenerateArtifactsVerb(VerbExtension):
    """Generate keys and permission files from a list of identities and policy files."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument('-k', '--keystore-root-path', help='root path of keystore')
        arg.completer = DirectoriesCompleter()
        parser.add_argument(
            '-e', '--enclaves', nargs='*', default=[],
            help='list of identities, aka ROS security enclave names')
        arg = parser.add_argument(
            '-p', '--policy-files', nargs='*', default=[],
            help='list of policy xml file paths')
        arg.completer = FilesCompleter(
            allowednames=('xml'), directories=False)

    def main(self, *, args):
        try:
            success = _artifact_generation.generate_artifacts(
                args.keystore_root_path, args.enclaves, args.policy_files)
        except FileNotFoundError as e:
            raise RuntimeError(str(e))
        return 0 if success else 1
