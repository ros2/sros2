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
from xml.etree import ElementTree

from cryptography import x509
from cryptography.hazmat.backends import default_backend as cryptography_backend
from cryptography.hazmat.primitives.serialization import load_pem_private_key

import pytest

from ros2cli import cli
from sros2.api import _DEFAULT_COMMON_NAME


# This fixture will run once for the entire module (as opposed to once per test)
@pytest.fixture(scope='module')
def keystore_dir(tmpdir_factory):
    keystore_dir = str(tmpdir_factory.mktemp('keystore'))

    # Create the keystore
    assert cli.main(argv=['security', 'create_keystore', keystore_dir]) == 0

    # Return path to keystore directory
    return keystore_dir


def test_create_keystore(keystore_dir):
    expected_files = (
        'ca.cert.pem', 'ca.key.pem', 'governance.p7s', 'governance.xml'
    )
    assert len(os.listdir(keystore_dir)) == len(expected_files)

    for expected_file in expected_files:
        assert os.path.isfile(os.path.join(keystore_dir, expected_file))


def test_ca_cert(keystore_dir):
    with open(os.path.join(keystore_dir, 'ca.cert.pem'), 'rb') as f:
        cert = x509.load_pem_x509_certificate(f.read(), cryptography_backend())
        names = cert.subject.get_attributes_for_oid(x509.oid.NameOID.COMMON_NAME)
        assert len(names) == 1
        assert names[0].value == _DEFAULT_COMMON_NAME
        names = cert.subject.get_attributes_for_oid(x509.oid.NameOID.ORGANIZATION_NAME)
        assert len(names) == 0


def test_ca_key(keystore_dir):
    with open(os.path.join(keystore_dir, 'ca.key.pem'), 'rb') as f:
        key = load_pem_private_key(f.read(), password=None, backend=cryptography_backend())
        public = key.public_key()
        assert public.curve.name == 'secp256r1'


def test_governance_p7s(keystore_dir):
    # Would really like to verify the signature, but ffi just can't use
    # that part of the OpenSSL API
    with open(os.path.join(keystore_dir, 'governance.p7s')) as f:
        lines = f.readlines()
        assert lines[0] == 'MIME-Version: 1.0\n'
        assert lines[1].startswith(
            'Content-Type: multipart/signed; protocol="application/x-pkcs7-signature"; micalg="sha-256";')  # noqa


def test_governance_xml(keystore_dir):
    # Validates valid XML
    ElementTree.parse(os.path.join(keystore_dir, 'governance.xml'))
