# Copyright 2019-2020 Canonical Ltd
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
import pathlib
from typing import Set

from cryptography import x509
from cryptography.hazmat.backends import default_backend as cryptography_backend
from cryptography.hazmat.primitives import serialization

from rclpy.exceptions import InvalidNamespaceException
from rclpy.validate_namespace import validate_namespace

from sros2 import _utilities
from sros2.api import _policy
import sros2.errors
from sros2.policy import get_policy_default

from . import _keystore, _permission


def create_enclave(keystore_path: pathlib.Path, identity: str) -> None:
    if not _keystore.is_valid_keystore(keystore_path):
        raise sros2.errors.InvalidKeystoreError(keystore_path)

    if not _is_enclave_name_valid(identity):
        raise sros2.errors.InvalidEnclaveNameError(identity)

    relative_path = os.path.normpath(identity.lstrip('/'))
    key_dir = _keystore.get_keystore_enclaves_dir(keystore_path).joinpath(relative_path)
    os.makedirs(key_dir, exist_ok=True)

    # symlink the CA cert in there
    public_certs = ['identity_ca.cert.pem', 'permissions_ca.cert.pem']
    for public_cert in public_certs:
        dst = key_dir.joinpath(public_cert)
        keystore_ca_cert_path = _keystore.get_keystore_public_dir(
            keystore_path).joinpath(public_cert)
        relativepath = pathlib.Path(
            os.path.relpath(keystore_ca_cert_path, key_dir))
        _utilities.create_symlink(src=relativepath, dst=dst)

    # symlink the governance file in there
    keystore_governance_path = _keystore.get_keystore_enclaves_dir(
        keystore_path).joinpath('governance.p7s')
    dest_governance_path = key_dir.joinpath('governance.p7s')
    relativepath = pathlib.Path(
        os.path.relpath(keystore_governance_path, key_dir))
    _utilities.create_symlink(src=relativepath, dst=dest_governance_path)

    keystore_identity_ca_cert_path = _keystore.get_keystore_public_dir(
        keystore_path).joinpath('identity_ca.cert.pem')
    keystore_identity_ca_key_path = _keystore.get_keystore_private_dir(
        keystore_path).joinpath('identity_ca.key.pem')

    # Only create certs/keys if they don't already exist
    cert_path = key_dir.joinpath('cert.pem')
    key_path = key_dir.joinpath('key.pem')
    if not cert_path.is_file() or not key_path.is_file():
        _create_key_and_cert(
            keystore_identity_ca_cert_path,
            keystore_identity_ca_key_path,
            identity,
            cert_path,
            key_path
        )

    # create a wildcard permissions file for this node which can be overridden
    # later using a policy if desired
    policy_file_path = get_policy_default('policy.xml')
    policy_element = _policy.get_policy('/', policy_file_path)
    enclave_element = policy_element.find('enclaves/enclave')
    enclave_element.attrib['path'] = identity

    permissions_path = key_dir.joinpath('permissions.xml')
    _permission.create_permission_file(permissions_path, _utilities.domain_id(), policy_element)

    signed_permissions_path = key_dir.joinpath('permissions.p7s')
    keystore_permissions_ca_cert_path = _keystore.get_keystore_public_dir(
        keystore_path).joinpath('permissions_ca.cert.pem')
    keystore_permissions_ca_key_path = _keystore.get_keystore_private_dir(
        keystore_path).joinpath('permissions_ca.key.pem')
    _utilities.create_smime_signed_file(
        keystore_permissions_ca_cert_path,
        keystore_permissions_ca_key_path,
        permissions_path,
        signed_permissions_path
    )


def get_enclaves(keystore_path: pathlib.Path) -> Set[str]:
    if not _keystore.is_valid_keystore(keystore_path):
        raise sros2.errors.InvalidKeystoreError(keystore_path)

    enclaves_path = _keystore.get_keystore_enclaves_dir(keystore_path)
    enclaves: Set[str] = set()
    if enclaves_path.is_dir():
        key_file_paths = enclaves_path.glob('**/key.pem')

        for key_file_path in key_file_paths:
            enclaves.add(f'/{key_file_path.parent.relative_to(enclaves_path).as_posix()}')

    return enclaves


def _is_enclave_name_valid(name: str) -> bool:
    # TODO(ivanpauno): Use validate_enclave_name when it's propagated to `rclpy`.
    #   This is not to bad for the moment.
    #   Related with https://github.com/ros2/rclpy/issues/528.
    try:
        return validate_namespace(name)
    except InvalidNamespaceException as e:
        print(e)
        return False


def _create_key_and_cert(
        keystore_ca_cert_path: pathlib.Path,
        keystore_ca_key_path: pathlib.Path,
        identity: str,
        cert_path: pathlib.Path,
        key_path: pathlib.Path):
    # Load the CA cert and key from disk
    ca_cert = _utilities.load_cert(keystore_ca_cert_path)

    with open(keystore_ca_key_path, 'rb') as f:
        ca_key = serialization.load_pem_private_key(f.read(), None, cryptography_backend())

    cert, private_key = _utilities.build_key_and_cert(
        x509.Name([x509.NameAttribute(x509.oid.NameOID.COMMON_NAME, identity)]),
        issuer_name=ca_cert.subject,
        ca_key=ca_key)

    _utilities.write_key(private_key, key_path)
    _utilities.write_cert(cert, cert_path)
