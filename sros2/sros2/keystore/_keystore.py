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

import pathlib

from cryptography import x509

import lxml

from sros2 import _utilities
import sros2.errors
from sros2.policy import get_transport_default, get_transport_schema


_KS_ENCLAVES = 'enclaves'
_KS_PUBLIC = 'public'
_KS_PRIVATE = 'private'
_DEFAULT_COMMON_NAME = 'sros2CA'


def create_keystore(keystore_path: pathlib.Path) -> None:
    if is_valid_keystore(keystore_path):
        raise sros2.errors.KeystoreExistsError(keystore_path)

    for path in (
            keystore_path,
            keystore_path.joinpath(_KS_PUBLIC),
            keystore_path.joinpath(_KS_PRIVATE),
            keystore_path.joinpath(_KS_ENCLAVES)):
        path.mkdir(parents=True, exist_ok=True)

    keystore_ca_cert_path = keystore_path.joinpath(_KS_PUBLIC, 'ca.cert.pem')
    keystore_ca_key_path = keystore_path.joinpath(_KS_PRIVATE, 'ca.key.pem')

    keystore_permissions_ca_cert_path = keystore_path.joinpath(
        _KS_PUBLIC, 'permissions_ca.cert.pem')
    keystore_permissions_ca_key_path = keystore_path.joinpath(
        _KS_PRIVATE, 'permissions_ca.key.pem')

    keystore_identity_ca_cert_path = keystore_path.joinpath(
        _KS_PUBLIC, 'identity_ca.cert.pem')
    keystore_identity_ca_key_path = keystore_path.joinpath(
        _KS_PRIVATE, 'identity_ca.key.pem')

    required_files = (
        keystore_permissions_ca_cert_path,
        keystore_permissions_ca_key_path,
        keystore_identity_ca_cert_path,
        keystore_identity_ca_key_path,
    )

    # Create new CA if one doesn't already exist
    if not all(x.is_file() for x in required_files):
        _create_ca_key_cert(keystore_ca_key_path, keystore_ca_cert_path)

        for path in (keystore_permissions_ca_cert_path, keystore_identity_ca_cert_path):
            _utilities.create_symlink(src=pathlib.Path('ca.cert.pem'), dst=path)

        for path in (keystore_permissions_ca_key_path, keystore_identity_ca_key_path):
            _utilities.create_symlink(src=pathlib.Path('ca.key.pem'), dst=path)

    # Create governance file if it doesn't already exist
    gov_path = keystore_path.joinpath(_KS_ENCLAVES, 'governance.xml')
    if not gov_path.is_file():
        _create_governance_file(gov_path, _utilities.domain_id())

    # Sign governance file if it hasn't already been signed
    signed_gov_path = keystore_path.joinpath(_KS_ENCLAVES, 'governance.p7s')
    if not signed_gov_path.is_file():
        _utilities.create_smime_signed_file(
            keystore_permissions_ca_cert_path,
            keystore_permissions_ca_key_path,
            gov_path,
            signed_gov_path)


def is_valid_keystore(path: pathlib.Path) -> bool:
    return (
        path.joinpath(_KS_PUBLIC, 'permissions_ca.cert.pem').is_file() and
        path.joinpath(_KS_PUBLIC, 'identity_ca.cert.pem').is_file() and
        path.joinpath(_KS_PRIVATE, 'permissions_ca.key.pem').is_file() and
        path.joinpath(_KS_PRIVATE, 'identity_ca.key.pem').is_file() and
        path.joinpath(_KS_ENCLAVES, 'governance.p7s').is_file()
    )


def get_keystore_enclaves_dir(keystore_path: pathlib.Path) -> pathlib.Path:
    return keystore_path.joinpath(_KS_ENCLAVES)


def get_keystore_public_dir(keystore_path: pathlib.Path) -> pathlib.Path:
    return keystore_path.joinpath(_KS_PUBLIC)


def get_keystore_private_dir(keystore_path: pathlib.Path) -> pathlib.Path:
    return keystore_path.joinpath(_KS_PRIVATE)


def _create_ca_key_cert(ca_key_out_path, ca_cert_out_path):
    cert, private_key = _utilities.build_key_and_cert(
        x509.Name([x509.NameAttribute(x509.oid.NameOID.COMMON_NAME, _DEFAULT_COMMON_NAME)]),
        ca=True)

    _utilities.write_key(private_key, ca_key_out_path)
    _utilities.write_cert(cert, ca_cert_out_path)


def _create_governance_file(path: pathlib.Path, domain_id: str):
    # for this application we are only looking to authenticate and encrypt;
    # we do not need/want access control at this point.
    governance_xml_path = get_transport_default('dds', 'governance.xml')
    governance_xml = lxml.etree.parse(str(governance_xml_path))

    governance_xsd_path = get_transport_schema('dds', 'governance.xsd')
    governance_xsd = lxml.etree.XMLSchema(lxml.etree.parse(str(governance_xsd_path)))

    domain_id_elements = governance_xml.findall(
        'domain_access_rules/domain_rule/domains/id')
    for domain_id_element in domain_id_elements:
        domain_id_element.text = domain_id

    try:
        governance_xsd.assertValid(governance_xml)
    except lxml.etree.DocumentInvalid as e:
        raise sros2.errors.InvalidGovernanceXMLError(e) from e

    with open(path, 'wb') as f:
        f.write(lxml.etree.tostring(governance_xml, pretty_print=True))
