# Copyright 2020 Canonical Ltd
# Copyright 2016-2019 Open Source Robotics Foundation, Inc.
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

from cryptography import x509

import lxml

from sros2.policy import get_transport_default, get_transport_schema

from . import _utilities


_KS_ENCLAVES = 'enclaves'
_KS_PUBLIC = 'public'
_KS_PRIVATE = 'private'
_DEFAULT_COMMON_NAME = 'sros2testCA'


def create_keystore(keystore_path):
    if not is_valid_keystore(keystore_path):
        print('creating keystore: %s' % keystore_path)
    else:
        print('keystore already exists: %s' % keystore_path)
        return

    os.makedirs(keystore_path, exist_ok=True)
    os.makedirs(os.path.join(keystore_path, _KS_PUBLIC), exist_ok=True)
    os.makedirs(os.path.join(keystore_path, _KS_PRIVATE), exist_ok=True)
    os.makedirs(os.path.join(keystore_path, _KS_ENCLAVES), exist_ok=True)

    keystore_ca_cert_path = os.path.join(keystore_path, _KS_PUBLIC, 'ca.cert.pem')
    keystore_ca_key_path = os.path.join(keystore_path, _KS_PRIVATE, 'ca.key.pem')

    keystore_permissions_ca_cert_path = os.path.join(
        keystore_path, _KS_PUBLIC, 'permissions_ca.cert.pem')
    keystore_permissions_ca_key_path = os.path.join(
        keystore_path, _KS_PRIVATE, 'permissions_ca.key.pem')
    keystore_identity_ca_cert_path = os.path.join(
        keystore_path, _KS_PUBLIC, 'identity_ca.cert.pem')
    keystore_identity_ca_key_path = os.path.join(
        keystore_path, _KS_PRIVATE, 'identity_ca.key.pem')

    required_files = (
        keystore_permissions_ca_cert_path,
        keystore_permissions_ca_key_path,
        keystore_identity_ca_cert_path,
        keystore_identity_ca_key_path,
    )

    if not all(os.path.isfile(x) for x in required_files):
        print('creating new CA key/cert pair')
        _create_ca_key_cert(keystore_ca_key_path, keystore_ca_cert_path)
        _utilities.create_symlink(src='ca.cert.pem', dst=keystore_permissions_ca_cert_path)
        _utilities.create_symlink(src='ca.key.pem', dst=keystore_permissions_ca_key_path)
        _utilities.create_symlink(src='ca.cert.pem', dst=keystore_identity_ca_cert_path)
        _utilities.create_symlink(src='ca.key.pem', dst=keystore_identity_ca_key_path)
    else:
        print('found CA key and cert, not creating new ones!')

    # create governance file
    gov_path = os.path.join(keystore_path, _KS_ENCLAVES, 'governance.xml')
    if not os.path.isfile(gov_path):
        print('creating governance file: %s' % gov_path)
        _create_governance_file(gov_path, _utilities.domain_id())
    else:
        print('found governance file, not creating a new one!')

    # sign governance file
    signed_gov_path = os.path.join(keystore_path, _KS_ENCLAVES, 'governance.p7s')
    if not os.path.isfile(signed_gov_path):
        print('creating signed governance file: %s' % signed_gov_path)
        _utilities.create_smime_signed_file(
            keystore_permissions_ca_cert_path,
            keystore_permissions_ca_key_path,
            gov_path,
            signed_gov_path)
    else:
        print('found signed governance file, not creating a new one!')

    print('all done! enjoy your keystore in %s' % keystore_path)
    print('cheers!')
    return True


def is_valid_keystore(path):
    return (
        os.path.isfile(os.path.join(path, _KS_PUBLIC, 'permissions_ca.cert.pem')) and
        os.path.isfile(os.path.join(path, _KS_PUBLIC, 'identity_ca.cert.pem')) and
        os.path.isfile(os.path.join(path, _KS_PRIVATE, 'permissions_ca.key.pem')) and
        os.path.isfile(os.path.join(path, _KS_PRIVATE, 'identity_ca.key.pem')) and
        os.path.isfile(os.path.join(path, _KS_ENCLAVES, 'governance.p7s'))
    )


def get_keystore_enclaves_dir(keystore_path: str) -> str:
    return os.path.join(keystore_path, _KS_ENCLAVES)


def get_keystore_public_dir(keystore_path: str) -> str:
    return os.path.join(keystore_path, _KS_PUBLIC)


def get_keystore_private_dir(keystore_path: str) -> str:
    return os.path.join(keystore_path, _KS_PRIVATE)


def _create_ca_key_cert(ca_key_out_path, ca_cert_out_path):
    cert, private_key = _utilities.build_key_and_cert(
        x509.Name([x509.NameAttribute(x509.oid.NameOID.COMMON_NAME, _DEFAULT_COMMON_NAME)]),
        ca=True)

    _utilities.write_key(private_key, ca_key_out_path)
    _utilities.write_cert(cert, ca_cert_out_path)


def _create_governance_file(path, domain_id):
    # for this application we are only looking to authenticate and encrypt;
    # we do not need/want access control at this point.
    governance_xml_path = get_transport_default('dds', 'governance.xml')
    governance_xml = lxml.etree.parse(governance_xml_path)

    governance_xsd_path = get_transport_schema('dds', 'governance.xsd')
    governance_xsd = lxml.etree.XMLSchema(lxml.etree.parse(governance_xsd_path))

    domain_id_elements = governance_xml.findall(
        'domain_access_rules/domain_rule/domains/id')
    for domain_id_element in domain_id_elements:
        domain_id_element.text = domain_id

    try:
        governance_xsd.assertValid(governance_xml)
    except lxml.etree.DocumentInvalid as e:
        raise RuntimeError(str(e))

    with open(path, 'wb') as f:
        f.write(lxml.etree.tostring(governance_xml, pretty_print=True))
