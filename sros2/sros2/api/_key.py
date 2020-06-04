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

import errno
import os
import pathlib

from cryptography import x509
from cryptography.hazmat.backends import default_backend as cryptography_backend
from cryptography.hazmat.primitives import serialization

from rclpy.exceptions import InvalidNamespaceException
from rclpy.validate_namespace import validate_namespace

from sros2.policy import get_policy_default

from . import _keystore, _permission, _policy, _utilities


def create_key(keystore_path, identity):
    if not _keystore.is_valid_keystore(keystore_path):
        print("'%s' is not a valid keystore " % keystore_path)
        return False
    if not _is_key_name_valid(identity):
        return False
    print("creating key for identity: '%s'" % identity)

    relative_path = os.path.normpath(identity.lstrip('/'))
    key_dir = os.path.join(_keystore.get_keystore_enclaves_dir(keystore_path), relative_path)
    os.makedirs(key_dir, exist_ok=True)

    # symlink the CA cert in there
    public_certs = ['identity_ca.cert.pem', 'permissions_ca.cert.pem']
    for public_cert in public_certs:
        dst = os.path.join(key_dir, public_cert)
        keystore_ca_cert_path = os.path.join(
            _keystore.get_keystore_public_dir(keystore_path), public_cert)
        relativepath = os.path.relpath(keystore_ca_cert_path, key_dir)
        _utilities.create_symlink(src=relativepath, dst=dst)

    # symlink the governance file in there
    keystore_governance_path = os.path.join(
        _keystore.get_keystore_enclaves_dir(keystore_path), 'governance.p7s')
    dest_governance_path = os.path.join(key_dir, 'governance.p7s')
    relativepath = os.path.relpath(keystore_governance_path, key_dir)
    _utilities.create_symlink(src=relativepath, dst=dest_governance_path)

    keystore_identity_ca_cert_path = os.path.join(
        _keystore.get_keystore_public_dir(keystore_path), 'identity_ca.cert.pem')
    keystore_identity_ca_key_path = os.path.join(
        _keystore.get_keystore_private_dir(keystore_path), 'identity_ca.key.pem')

    cert_path = os.path.join(key_dir, 'cert.pem')
    key_path = os.path.join(key_dir, 'key.pem')
    if not os.path.isfile(cert_path) or not os.path.isfile(key_path):
        print('creating cert and key')
        _create_key_and_cert(
            keystore_identity_ca_cert_path,
            keystore_identity_ca_key_path,
            identity,
            cert_path,
            key_path
        )
    else:
        print('found cert and key; not creating new ones!')

    # create a wildcard permissions file for this node which can be overridden
    # later using a policy if desired
    policy_file_path = get_policy_default('policy.xml')
    policy_element = _policy.get_policy('/', policy_file_path)
    enclave_element = policy_element.find('enclaves/enclave')
    enclave_element.attrib['path'] = identity

    permissions_path = os.path.join(key_dir, 'permissions.xml')
    _permission.create_permission_file(permissions_path, _utilities.domain_id(), policy_element)

    signed_permissions_path = os.path.join(key_dir, 'permissions.p7s')
    keystore_permissions_ca_cert_path = os.path.join(
        _keystore.get_keystore_public_dir(keystore_path), 'permissions_ca.cert.pem')
    keystore_permissions_ca_key_path = os.path.join(
        _keystore.get_keystore_private_dir(keystore_path), 'permissions_ca.key.pem')
    _utilities.create_smime_signed_file(
        keystore_permissions_ca_cert_path,
        keystore_permissions_ca_key_path,
        permissions_path,
        signed_permissions_path
    )

    return True


def list_keys(keystore_path):
    enclaves_path = _keystore.get_keystore_enclaves_dir(keystore_path)
    if not os.path.isdir(keystore_path):
        raise FileNotFoundError(
            errno.ENOENT, os.strerror(errno.ENOENT), keystore_path)
    if not os.path.isdir(enclaves_path):
        return True
    p = pathlib.Path(enclaves_path)
    key_file_paths = sorted(p.glob('**/key.pem'))
    for key_file_path in key_file_paths:
        print('/{}'.format(key_file_path.parent.relative_to(enclaves_path).as_posix()))
    return True


def _is_key_name_valid(name):
    # TODO(ivanpauno): Use validate_enclave_name when it's propagated to `rclpy`.
    #   This is not to bad for the moment.
    #   Related with https://github.com/ros2/rclpy/issues/528.
    try:
        return validate_namespace(name)
    except InvalidNamespaceException as e:
        print(e)
        return False


def _create_key_and_cert(
        keystore_ca_cert_path, keystore_ca_key_path, identity, cert_path, key_path):
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
