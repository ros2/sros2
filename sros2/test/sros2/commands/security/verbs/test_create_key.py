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

import configparser
import os
import textwrap
from xml.etree import ElementTree

import cryptography
from cryptography import x509
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import ec

import pytest

from ros2cli import cli
from sros2.api import create_keystore


# This fixture will run once for the entire module (as opposed to once per test)
@pytest.fixture(scope='module')
def node_keys_dir(tmp_path_factory):
    keystore_dir = str(tmp_path_factory.mktemp('keystore'))

    # First, create the keystore
    assert create_keystore(keystore_dir)

    # Now using that keystore, create a keypair along with other files required by DDS
    assert cli.main(argv=['security', 'create_key', keystore_dir, '/test_node']) == 0
    node_dir = os.path.join(keystore_dir, 'test_node')
    assert os.path.isdir(os.path.join(keystore_dir, 'test_node'))

    # Return path to directory containing the node's files
    return node_dir


def load_cert(path):
    with open(path, 'rb') as f:
        pem_data = f.read()
    return x509.load_pem_x509_certificate(pem_data, default_backend())


def load_csr(path):
    with open(path, 'rb') as f:
        pem_data = f.read()
    return x509.load_pem_x509_csr(pem_data, default_backend())


def load_private_key(path):
    with open(path, 'rb') as f:
        pem_data = f.read()
    return serialization.load_pem_private_key(pem_data, password=None, backend=default_backend())


def check_common_name(entity, expected_value):
    names = entity.get_attributes_for_oid(x509.oid.NameOID.COMMON_NAME)
    assert len(names) == 1
    assert names[0].value == expected_value


def verify_signature(cert, signatory):
    try:
        signatory.public_key().verify(
            cert.signature,
            cert.tbs_certificate_bytes,
            ec.ECDSA(cert.signature_hash_algorithm))
    except cryptography.exceptions.InvalidSignature:
        return False
    return True


def test_create_key(node_keys_dir):
    expected_files = (
        'cert.pem', 'ecdsaparam', 'governance.p7s', 'identity_ca.cert.pem', 'key.pem',
        'permissions.p7s', 'permissions.xml', 'permissions_ca.cert.pem', 'req.pem', 'request.cnf'
    )
    assert len(os.listdir(node_keys_dir)) == len(expected_files)

    for expected_file in expected_files:
        assert os.path.isfile(os.path.join(node_keys_dir, expected_file))


def test_cert_pem(node_keys_dir):
    cert = load_cert(os.path.join(node_keys_dir, 'cert.pem'))
    check_common_name(cert.subject, u'/test_node')
    check_common_name(cert.issuer, u'sros2testCA')

    signatory = load_cert(os.path.join(node_keys_dir, 'identity_ca.cert.pem'))
    assert verify_signature(cert, signatory)


def test_ecdsaparam(node_keys_dir):
    with open(os.path.join(node_keys_dir, 'ecdsaparam')) as f:
        assert f.read() == textwrap.dedent("""\
            -----BEGIN EC PARAMETERS-----
            BggqhkjOPQMBBw==
            -----END EC PARAMETERS-----
            """)


def test_governance_p7s(node_keys_dir):
    # Would really like to verify the signature, but ffi just can't use
    # that part of the OpenSSL API
    with open(os.path.join(node_keys_dir, 'governance.p7s')) as f:
        lines = f.readlines()
        assert lines[0] == 'MIME-Version: 1.0\n'
        assert lines[1].startswith(
            'Content-Type: multipart/signed; protocol="application/x-pkcs7-signature"; micalg="sha-256";')  # noqa


def test_identity_ca_cert_pem(node_keys_dir):
    cert = load_cert(os.path.join(node_keys_dir, 'identity_ca.cert.pem'))
    check_common_name(cert.subject, u'sros2testCA')
    check_common_name(cert.issuer, u'sros2testCA')


def test_key_pem(node_keys_dir):
    private_key = load_private_key(os.path.join(node_keys_dir, 'key.pem'))
    public_key = private_key.public_key()
    assert isinstance(public_key.curve, ec.SECP256R1)


def test_permissions_p7s(node_keys_dir):
    # Would really like to verify the signature, but ffi just can't use
    # that part of the OpenSSL API
    with open(os.path.join(node_keys_dir, 'permissions.p7s')) as f:
        lines = f.readlines()
        assert lines[0] == 'MIME-Version: 1.0\n'
        assert lines[1].startswith(
            'Content-Type: multipart/signed; protocol="application/x-pkcs7-signature"; micalg="sha-256";')  # noqa


def test_permissions_xml(node_keys_dir):
    ElementTree.parse(os.path.join(node_keys_dir, 'permissions.xml'))


def test_permissions_ca_cert_pem(node_keys_dir):
    cert = load_cert(os.path.join(node_keys_dir, 'permissions_ca.cert.pem'))
    check_common_name(cert.subject, u'sros2testCA')
    check_common_name(cert.issuer, u'sros2testCA')

    signatory = load_cert(os.path.join(node_keys_dir, 'identity_ca.cert.pem'))
    assert verify_signature(cert, signatory)


def test_req_pem(node_keys_dir):
    csr = load_csr(os.path.join(node_keys_dir, 'req.pem'))
    check_common_name(csr.subject, u'/test_node')


def test_request_cnf(node_keys_dir):
    config = configparser.ConfigParser()

    # ConfigParser doesn't support INI files without section headers, so pretend one
    # is there
    with open(os.path.join(node_keys_dir, 'request.cnf')) as f:
        config.read_string('[root]\n' + f.read())

    for expected_section in ('root', ' req_distinguished_name '):
        assert expected_section in config.sections()

    root_config = config['root']
    for expected_root_key in ('prompt', 'string_mask', 'distinguished_name'):
        assert expected_root_key in root_config

    assert root_config['prompt'] == 'no'
    assert root_config['string_mask'] == 'utf8only'
    assert root_config['distinguished_name'] == 'req_distinguished_name'

    req_config = config[' req_distinguished_name ']
    assert 'commonName' in req_config
    assert req_config['commonName'] == '/test_node'
