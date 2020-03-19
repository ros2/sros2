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
from pathlib import Path
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
    assert cli.main(argv=['security', 'create_keystore', '-k', keystore_dir]) == 0

    # Return path to keystore directory
    return Path(keystore_dir)


def test_create_keystore(keystore_dir):
    public = keystore_dir / 'public'
    private = keystore_dir / 'private'
    contexts = keystore_dir / 'contexts'
    expected_files_public = (
        public / 'ca.cert.pem',
        public / 'permissions_ca.cert.pem',
        public / 'identity_ca.cert.pem',
    )
    expected_files_private = (
        private / 'ca.key.pem',
        private / 'permissions_ca.key.pem',
        private / 'identity_ca.key.pem',
    )
    expected_files_contexts = (
        contexts / 'governance.p7s',
        contexts / 'governance.xml',
    )

    assert len(list(keystore_dir.iterdir())) == 3
    assert len(list(public.iterdir())) == len(expected_files_public)
    assert len(list(private.iterdir())) == len(expected_files_private)
    assert len(list(contexts.iterdir())) == len(expected_files_contexts)
    expected_files = expected_files_public + expected_files_private + expected_files_contexts
    assert all(x.exists() for x in expected_files)


def test_ca_cert(keystore_dir):
    with (keystore_dir / 'public' / 'ca.cert.pem').open('rb') as f:
        cert = x509.load_pem_x509_certificate(f.read(), cryptography_backend())
        names = cert.subject.get_attributes_for_oid(x509.oid.NameOID.COMMON_NAME)
        assert len(names) == 1
        assert names[0].value == _DEFAULT_COMMON_NAME
        names = cert.subject.get_attributes_for_oid(x509.oid.NameOID.ORGANIZATION_NAME)
        assert len(names) == 0


def test_ca_key(keystore_dir):
    with (keystore_dir / 'private' / 'ca.key.pem').open('rb') as f:
        key = load_pem_private_key(f.read(), password=None, backend=cryptography_backend())
        public = key.public_key()
        assert public.curve.name == 'secp256r1'


def test_governance_p7s(keystore_dir):
    # Would really like to verify the signature, but ffi just can't use
    # that part of the OpenSSL API
    with (keystore_dir / 'contexts' / 'governance.p7s').open('r') as f:
        lines = f.readlines()
        assert lines[0] == 'MIME-Version: 1.0\n'
        assert lines[1].startswith(
            'Content-Type: multipart/signed; protocol="application/x-pkcs7-signature"; micalg="sha-256";')  # noqa


def test_governance_xml(keystore_dir):
    # Validates valid XML
    ElementTree.parse(str(keystore_dir / 'contexts' / 'governance.xml'))
