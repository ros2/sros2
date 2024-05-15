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

import datetime
from pathlib import Path

import cryptography
from cryptography import x509
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec

from lxml import etree

import pytest

from ros2cli import cli

from sros2 import _utilities
import sros2.keystore
from sros2.policy import get_transport_schema


# This fixture will run once for the entire module (as opposed to once per test)
@pytest.fixture(scope='module')
def enclave_keys_dir(tmp_path_factory) -> Path:
    keystore_dir = tmp_path_factory.mktemp('keystore')

    # First, create the keystore
    sros2.keystore.create_keystore(keystore_dir)
    assert keystore_dir.is_dir()

    # Now using that keystore, create a keypair along with other files required by DDS
    assert cli.main(
        argv=['security', 'create_enclave', str(keystore_dir), '/test_enclave']) == 0
    enclave_dir = keystore_dir / 'enclaves' / 'test_enclave'
    assert enclave_dir.is_dir()

    # Return path to directory containing the enclave's files
    return enclave_dir


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


def _datetimes_are_close(actual, expected):
    # We can't check exact times, but an hour's resolution is fine for testing purposes
    return actual <= expected and actual >= (expected - datetime.timedelta(hours=1))


def verify_signature(cert, signatory):
    try:
        signatory.public_key().verify(
            cert.signature,
            cert.tbs_certificate_bytes,
            ec.ECDSA(cert.signature_hash_algorithm))
    except cryptography.exceptions.InvalidSignature:
        return False
    return True


def test_create_enclave(enclave_keys_dir):
    expected_files = (
        'cert.pem', 'governance.p7s', 'identity_ca.cert.pem', 'key.pem', 'permissions.p7s',
        'permissions.xml', 'permissions_ca.cert.pem'
    )
    assert len(list(enclave_keys_dir.iterdir())) == len(expected_files)

    for expected_file in expected_files:
        assert (enclave_keys_dir / expected_file).is_file()


def test_create_enclave_twice(tmp_path):
    # First, create the keystore
    sros2.keystore.create_keystore(tmp_path)
    assert tmp_path.is_dir()
    keystore_dir = tmp_path

    # Now using that keystore, create an enclave
    assert cli.main(
        argv=['security', 'create_enclave', str(keystore_dir), '/test_enclave']) == 0
    enclave_dir = keystore_dir / 'enclaves' / 'test_enclave'
    assert enclave_dir.is_dir()

    # Now create it again and confirm that the command doesn't fail
    assert cli.main(
        argv=['security', 'create_enclave', str(keystore_dir), '/test_enclave']) == 0


def test_cert_pem(enclave_keys_dir):
    cert = _utilities.load_cert(enclave_keys_dir / 'cert.pem')
    check_common_name(cert.subject, u'/test_enclave')
    check_common_name(cert.issuer, sros2.keystore._keystore._DEFAULT_COMMON_NAME)

    # Verify that the hash algorithm is as expected
    assert isinstance(cert.signature_hash_algorithm, hashes.SHA256)

    # Verify the cert is valid for the expected timespan
    utcnow = datetime.datetime.utcnow()

    # Using a day earlier here to prevent Connext (5.3.1) from complaining
    # when extracting it from the permissions file and thinking it's in the future
    # https://github.com/ros2/ci/pull/436#issuecomment-624874296
    assert _datetimes_are_close(cert.not_valid_before, utcnow - datetime.timedelta(days=1))
    assert _datetimes_are_close(cert.not_valid_after, utcnow + datetime.timedelta(days=3650))

    # Verify that the cert ensures this key cannot be used to sign others as a CA
    assert len(cert.extensions) == 1
    extension = cert.extensions[0]
    assert extension.critical is False
    value = extension.value
    assert isinstance(value, x509.BasicConstraints)
    assert value.ca is False
    assert value.path_length is None

    # Verify this cert is indeed signed by the keystore CA
    signatory = _utilities.load_cert(enclave_keys_dir / 'identity_ca.cert.pem')
    assert verify_signature(cert, signatory)


def test_governance_p7s(enclave_keys_dir):
    # Would really like to verify the signature, but ffi just can't use
    # that part of the OpenSSL API
    with open(enclave_keys_dir / 'governance.p7s') as f:
        lines = f.readlines()
        assert lines[0] == 'MIME-Version: 1.0\n'
        assert lines[1].startswith(
            'Content-Type: multipart/signed; protocol="application/x-pkcs7-signature"; micalg="sha-256";')  # noqa


def test_identity_ca_cert_pem(enclave_keys_dir):
    cert = _utilities.load_cert(enclave_keys_dir / 'identity_ca.cert.pem')
    check_common_name(cert.subject, sros2.keystore._keystore._DEFAULT_COMMON_NAME)
    check_common_name(cert.issuer, sros2.keystore._keystore._DEFAULT_COMMON_NAME)


def test_key_pem(enclave_keys_dir):
    private_key = load_private_key(enclave_keys_dir / 'key.pem')
    assert isinstance(private_key, ec.EllipticCurvePrivateKey)
    assert private_key.key_size == 256

    public_key = private_key.public_key()
    assert isinstance(public_key.curve, ec.SECP256R1)
    assert public_key.key_size == 256


def test_permissions_p7s(enclave_keys_dir):
    # Would really like to verify the signature, but ffi just can't use
    # that part of the OpenSSL API
    with open(enclave_keys_dir / 'permissions.p7s') as f:
        lines = f.readlines()
        assert lines[0] == 'MIME-Version: 1.0\n'
        assert lines[1].startswith(
            'Content-Type: multipart/signed; protocol="application/x-pkcs7-signature"; micalg="sha-256";')  # noqa


def test_permissions_xml(enclave_keys_dir):
    permissions_xml = etree.parse(str(enclave_keys_dir / 'permissions.xml'))
    permissions_xsd_path = get_transport_schema('dds', 'permissions.xsd')
    permissions_xsd = etree.XMLSchema(etree.parse(str(permissions_xsd_path)))
    permissions_xsd.assertValid(permissions_xml)


def test_permissions_ca_cert_pem(enclave_keys_dir):
    cert = _utilities.load_cert(enclave_keys_dir / 'permissions_ca.cert.pem')
    check_common_name(cert.subject, sros2.keystore._keystore._DEFAULT_COMMON_NAME)
    check_common_name(cert.issuer, sros2.keystore._keystore._DEFAULT_COMMON_NAME)

    signatory = _utilities.load_cert(enclave_keys_dir / 'identity_ca.cert.pem')
    assert verify_signature(cert, signatory)
