# Copyright 2024 Mikael Arguedas
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

from ros2cli import cli


def test_no_verb(capsys):
    assert cli.main(argv=['security']) == 0
    output = capsys.readouterr().out.rstrip()
    assert 'Call `ros2 security <command> -h` for more detailed usage.' in output
