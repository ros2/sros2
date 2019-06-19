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
import tempfile
from xml.etree import ElementTree

from cryptography import x509
from cryptography.hazmat.backends import default_backend as cryptography_backend
from cryptography.hazmat.primitives.serialization import load_pem_private_key

from ros2cli import cli


def test_create_keystore():
    def check_index_txt(path):
        with open(path, 'r') as f:
            lines = f.readlines()
            assert len(lines) == 0

    def check_ca_cert_pem(path):
        with open(path, 'rb') as f:
            cert = x509.load_pem_x509_certificate(f.read(), cryptography_backend())
            names = cert.subject.get_attributes_for_oid(x509.oid.NameOID.COMMON_NAME)
            assert len(names) == 1
            assert names[0].value == u'sros2testCA'
            names = cert.subject.get_attributes_for_oid(x509.oid.NameOID.ORGANIZATION_NAME)
            assert len(names) == 0

    def check_ca_conf(path):
        config = configparser.ConfigParser()
        successful_reads = config.read(path)
        assert len(successful_reads) == 1
        assert config.sections() == [
            ' ca ',
            ' CA_default ',
            ' policy_match ',
            ' local_ca_extensions ',
            ' req ',
            ' req_distinguished_name ',
            ' root_ca_extensions ',
        ]

    def check_ecdsaparam(path):
        with open(path, 'r') as f:
            # cryptography does not seem to know how to load ecparams
            lines = f.readlines()
            assert lines[0] == '-----BEGIN EC PARAMETERS-----\n'
            assert lines[-1] == '-----END EC PARAMETERS-----\n'

    def check_governance_xml(path):
        # validates valid XML
        ElementTree.parse(path)

    def check_ca_key_pem(path):
        with open(path, 'rb') as f:
            key = load_pem_private_key(f.read(), password=None, backend=cryptography_backend())
            public = key.public_key()
            assert public.curve.name == 'secp256r1'

    with tempfile.TemporaryDirectory() as keystore_dir:
        assert cli.main(argv=['security', 'create_keystore', keystore_dir]) == 0
        expected_files = (
            ('governance.p7s', None),
            ('index.txt', check_index_txt),
            ('ca.cert.pem', check_ca_cert_pem),
            ('ca_conf.cnf', check_ca_conf),
            ('ecdsaparam', check_ecdsaparam),
            ('governance.xml', check_governance_xml),
            ('ca.key.pem', check_ca_key_pem),
            ('serial', None),
        )

        for expected_file, file_validator in expected_files:
            path = os.path.join(keystore_dir, expected_file)
            assert os.path.isfile(path), 'Expected output file %s was not found.' % expected_file
            if file_validator:
                file_validator(path)
