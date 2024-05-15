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

from pathlib import Path

import pytest

from ros2cli import cli

from sros2 import _utilities


# Here we provide only very high level testing as this verb
# is just a combination of calls to the others ones covered by precise tests

# This fixture will run once for the entire module (as opposed to once per test)
@pytest.fixture(scope='module')
def keystore_dir(tmp_path_factory) -> Path:
    keystore_dir = tmp_path_factory.mktemp('keystore')

    # Create the keystore
    assert cli.main(argv=['security', 'create_keystore', str(keystore_dir)]) == 0

    # Return path to keystore directory
    return keystore_dir


def test_cli_keystore_args(capsys, tmp_path, monkeypatch, keystore_dir):
    # invalid keystore
    assert cli.main(argv=['security', 'generate_artifacts', '-k', str(tmp_path)]) == 0
    output = capsys.readouterr().out.rstrip()
    assert 'is not a valid keystore, creating new keystore' in output

    assert cli.main(argv=['security', 'generate_artifacts', '-k', str(keystore_dir)]) == 0

    # keystore from env
    with monkeypatch.context() as m:
        m.setenv(_utilities._KEYSTORE_DIR_ENV, str(keystore_dir))
        assert cli.main(argv=['security', 'generate_artifacts']) == 0

    # invalid keystore from env
    tmp_keystore_folder = tmp_path
    with monkeypatch.context() as m:
        m.setenv(_utilities._KEYSTORE_DIR_ENV, str(tmp_keystore_folder / 'bar'))
        assert cli.main(argv=['security', 'generate_artifacts']) == 0
        output = capsys.readouterr().out.rstrip()
        assert 'is not a valid keystore, creating new keystore' in output

    # no keystore in args or in env
    with monkeypatch.context() as m:
        m.delenv(_utilities._KEYSTORE_DIR_ENV, raising=False)
        assert cli.main(argv=['security', 'generate_artifacts']) == 1
        output = capsys.readouterr().err.rstrip()
        assert (
            'Unable to generate artifacts: '
            "'ROS_SECURITY_KEYSTORE' isn't pointing at a valid keystore"
            in output
        )


def test_cli_enclave_args(keystore_dir):
    # no enclaves
    assert cli.main(argv=['security', 'generate_artifacts', '-k', str(keystore_dir)]) == 0

    # 1 existing enclave and 1 to create
    assert cli.main(
        argv=['security', 'create_enclave', str(keystore_dir), '/test_enclave']) == 0
    enclave_list = ['/test_enclave', '/test_enclave2']
    command_args = ['security', 'generate_artifacts', '-k', str(keystore_dir)]
    for name in enclave_list:
        command_args.append('-e')
        command_args.append(name)
    assert cli.main(argv=command_args) == 0
    expected_files = (
        'cert.pem', 'governance.p7s', 'identity_ca.cert.pem', 'key.pem', 'permissions.p7s',
        'permissions.xml', 'permissions_ca.cert.pem'
    )
    for enclave in enclave_list:
        enclave_keys_dir = keystore_dir / 'enclaves' / enclave.lstrip('/')
        assert len(list(enclave_keys_dir.iterdir())) == len(expected_files)

        for expected_file in expected_files:
            assert (enclave_keys_dir / expected_file).is_file()


def test_cli_policies_args(capsys, keystore_dir, test_policy_dir):
    enclave_list = ['/test_enclave', '/test_enclave2', '/minimal_action/minimal_action_server']
    command_args = ['security', 'generate_artifacts', '-k', str(keystore_dir)]
    for name in enclave_list:
        command_args.append('-e')
        command_args.append(name)
    # Test an invalid policy file
    retcode = cli.main(
        argv=command_args + [
            '-p', str(test_policy_dir / 'invalid_policy_missing_topics_tag.xml')
        ]
    )
    assert "Element 'topic': This element is not expected." in retcode
    # Test a valid policy file
    assert cli.main(
        argv=command_args + [
            '-p', str(test_policy_dir / 'minimal_action.policy.xml')
        ]
    ) == 0
    # ensure that missing enclaves have been created on the fly
    for name in enclave_list:
        assert Path(keystore_dir / 'enclaves' / name.lstrip('/')).is_dir()
    # Test a valid set of policy files
    assert cli.main(
        argv=command_args + [
            '-p', str(test_policy_dir / 'minimal_action.policy.xml'),
            '-p', str(test_policy_dir / 'add_two_ints.policy.xml'),
            '-p', str(test_policy_dir / 'talker_listener.policy.xml'),
        ]
    ) == 0
