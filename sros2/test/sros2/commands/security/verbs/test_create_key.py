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
import tempfile
from xml.etree import ElementTree

from cryptography import x509
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import ec

from ros2cli import cli


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


def check_cert_pem(path):
    cert = load_cert(path)
    check_common_name(cert.subject, u'/test_node')
    check_common_name(cert.issuer, u'sros2testCA')


def check_permissions_xml(path):
    ElementTree.parse(path)


def check_permissions_ca_cert_pem(path):
    cert = load_cert(path)
    check_common_name(cert.subject, u'sros2testCA')
    check_common_name(cert.issuer, u'sros2testCA')


def check_req_pem(path):
    csr = load_csr(path)
    check_common_name(csr.subject, u'/test_node')


def check_key_pem(path):
    private_key = load_private_key(path)
    public_key = private_key.public_key()
    assert isinstance(public_key.curve, ec.SECP256R1)


def check_identity_ca_cert_pem(path):
    cert = load_cert(path)
    check_common_name(cert.subject, u'sros2testCA')
    check_common_name(cert.issuer, u'sros2testCA')


def test_create_key():
    with tempfile.TemporaryDirectory() as keystore_dir:
        # First, create the keystore
        assert cli.main(argv=['security', 'create_keystore', keystore_dir]) == 0

        # Now using that keystore, create a keypair
        assert cli.main(argv=['security', 'create_key', keystore_dir, '/test_node']) == 0

        assert os.path.isdir(os.path.join(keystore_dir, 'test_node'))

        expected_files = (
            ('cert.pem', check_cert_pem),
            ('permissions.xml', check_permissions_xml),
            ('permissions_ca.cert.pem', check_permissions_ca_cert_pem),
            ('request.cnf', None),
            ('req.pem', check_req_pem),
            ('permissions.p7s', None),
            ('key.pem', check_key_pem),
            ('identity_ca.cert.pem', check_identity_ca_cert_pem),
            ('governance.p7s', None),
            ('ecdsaparam', None),
        )
        for expected_file, file_validator in expected_files:
            path = os.path.join(keystore_dir, 'test_node', expected_file)
            assert os.path.isfile(path)
            if file_validator:
                file_validator(path)
