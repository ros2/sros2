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

from collections import namedtuple
import datetime
import os
import shutil
import sys

from cryptography import x509
from cryptography.hazmat.backends import default_backend as cryptography_backend
from cryptography.hazmat.bindings.openssl.binding import Binding as SSLBinding
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import ec

from lxml import etree

from rclpy.exceptions import InvalidNamespaceException
from rclpy.exceptions import InvalidNodeNameException
from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name

from sros2.policy import (
    get_policy_default,
    get_transport_default,
    get_transport_schema,
    get_transport_template,
    load_policy,
)

HIDDEN_NODE_PREFIX = '_'
DOMAIN_ID_ENV = 'ROS_DOMAIN_ID'

_DEFAULT_COMMON_NAME = 'sros2testCA'

NodeName = namedtuple('NodeName', ('node', 'ns', 'fqn'))
TopicInfo = namedtuple('Topic', ('fqn', 'type'))


def get_node_names(*, node, include_hidden_nodes=False):
    node_names_and_namespaces = node.get_node_names_and_namespaces()
    return [
        NodeName(
            node=t[0],
            ns=t[1],
            fqn=t[1] + ('' if t[1].endswith('/') else '/') + t[0])
        for t in node_names_and_namespaces
        if (
            include_hidden_nodes or
            (t[0] and not t[0].startswith(HIDDEN_NODE_PREFIX))
        )
    ]


def get_topics(node_name, func):
    names_and_types = func(node_name.node, node_name.ns)
    return [
        TopicInfo(
            fqn=t[0],
            type=t[1])
        for t in names_and_types]


def get_subscriber_info(node, node_name):
    return get_topics(node_name, node.get_subscriber_names_and_types_by_node)


def get_publisher_info(node, node_name):
    return get_topics(node_name, node.get_publisher_names_and_types_by_node)


def get_service_info(node, node_name):
    return get_topics(node_name, node.get_service_names_and_types_by_node)


def get_client_info(node, node_name):
    return get_topics(node_name, node.get_client_names_and_types_by_node)


def _write_key(
    key,
    key_path,
    *,
    encoding=serialization.Encoding.PEM,
    serialization_format=serialization.PrivateFormat.PKCS8,
    encryption_algorithm=serialization.NoEncryption()
):
    with open(key_path, 'wb') as f:
        f.write(key.private_bytes(
            encoding=encoding,
            format=serialization_format,
            encryption_algorithm=encryption_algorithm))


def _write_cert(cert, cert_path, *, encoding=serialization.Encoding.PEM):
    with open(cert_path, 'wb') as f:
        f.write(cert.public_bytes(encoding=encoding))


def create_ca_key_cert(ca_key_out_path, ca_cert_out_path):
    cert, private_key = _build_key_and_cert(
        x509.Name([x509.NameAttribute(x509.oid.NameOID.COMMON_NAME, _DEFAULT_COMMON_NAME)]),
        ca=True)

    _write_key(private_key, ca_key_out_path)
    _write_cert(cert, ca_cert_out_path)


def create_governance_file(path, domain_id):
    # for this application we are only looking to authenticate and encrypt;
    # we do not need/want access control at this point.
    governance_xml_path = get_transport_default('dds', 'governance.xml')
    governance_xml = etree.parse(governance_xml_path)

    governance_xsd_path = get_transport_schema('dds', 'governance.xsd')
    governance_xsd = etree.XMLSchema(etree.parse(governance_xsd_path))

    domain_id_elements = governance_xml.findall(
        'domain_access_rules/domain_rule/domains/id')
    for domain_id_element in domain_id_elements:
        domain_id_element.text = domain_id

    try:
        governance_xsd.assertValid(governance_xml)
    except etree.DocumentInvalid as e:
        raise RuntimeError(str(e))

    with open(path, 'wb') as f:
        f.write(etree.tostring(governance_xml, pretty_print=True))


def create_keystore(keystore_path):
    if not os.path.exists(keystore_path):
        print('creating keystore: %s' % keystore_path)
        os.makedirs(keystore_path, exist_ok=True)
        os.makedirs(os.path.join(keystore_path, 'public'), exist_ok=True)
        os.makedirs(os.path.join(keystore_path, 'private'), exist_ok=True)
        os.makedirs(os.path.join(keystore_path, 'contexts'), exist_ok=True)
    else:
        print('keystore already exists: %s' % keystore_path)

    keystore_ca_cert_path = os.path.join(keystore_path, 'public', 'ca.cert.pem')
    keystore_ca_key_path = os.path.join(keystore_path, 'private', 'ca.key.pem')
    
    keystore_permissions_ca_cert_path = os.path.join(keystore_path, 'public', 'permissions_ca.cert.pem')
    keystore_permissions_ca_key_path = os.path.join(keystore_path, 'private', 'permissions_ca.key.pem')
    keystore_identity_ca_cert_path = os.path.join(keystore_path, 'public', 'identity_ca.cert.pem')
    keystore_identity_ca_key_path = os.path.join(keystore_path, 'private', 'identity_ca.key.pem')

    if not (os.path.isfile(keystore_permissions_ca_cert_path) and os.path.isfile(keystore_permissions_ca_key_path) and
       not (os.path.isfile(keystore_identity_ca_cert_path) and os.path.isfile(keystore_identity_ca_key_path))):
        print('creating new CA key/cert pair')
        create_ca_key_cert(keystore_ca_key_path, keystore_ca_cert_path)
        os.symlink(src='ca.cert.pem', dst=keystore_permissions_ca_cert_path)
        os.symlink(src='ca.key.pem', dst=keystore_permissions_ca_key_path)
        os.symlink(src='ca.cert.pem', dst=keystore_identity_ca_cert_path)
        os.symlink(src='ca.key.pem', dst=keystore_identity_ca_key_path)
    else:
        print('found CA key and cert, not creating new ones!')

    # create governance file
    gov_path = os.path.join(keystore_path, 'contexts', 'governance.xml')
    if not os.path.isfile(gov_path):
        print('creating governance file: %s' % gov_path)
        domain_id = os.getenv(DOMAIN_ID_ENV, '0')
        create_governance_file(gov_path, domain_id)
    else:
        print('found governance file, not creating a new one!')

    # sign governance file
    signed_gov_path = os.path.join(keystore_path, 'contexts', 'governance.p7s')
    if not os.path.isfile(signed_gov_path):
        print('creating signed governance file: %s' % signed_gov_path)
        _create_smime_signed_file(
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
        os.path.isfile(os.path.join(path, 'public', 'permissions_ca.cert.pem')) and
        os.path.isfile(os.path.join(path, 'public', 'identity_ca.cert.pem')) and
        os.path.isfile(os.path.join(path, 'private','permissions_ca.key.pem')) and
        os.path.isfile(os.path.join(path, 'private','identity_ca.key.pem')) and
        os.path.isfile(os.path.join(path, 'contexts', 'governance.p7s'))
    )


def is_key_name_valid(name):
    ns_and_name = name.rsplit('/', 1)
    if len(ns_and_name) != 2:
        print("The key name needs to start with '/'")
        return False
    node_ns = ns_and_name[0] if ns_and_name[0] else '/'
    node_name = ns_and_name[1]

    try:
        return (validate_namespace(node_ns) and validate_node_name(node_name))
    except (InvalidNamespaceException, InvalidNodeNameException) as e:
        print('{}'.format(e))
        return False


def create_permission_file(path, domain_id, policy_element):
    permissions_xsl_path = get_transport_template('dds', 'permissions.xsl')
    permissions_xsl = etree.XSLT(etree.parse(permissions_xsl_path))
    permissions_xsd_path = get_transport_schema('dds', 'permissions.xsd')
    permissions_xsd = etree.XMLSchema(etree.parse(permissions_xsd_path))

    permissions_xml = permissions_xsl(policy_element)

    domain_id_elements = permissions_xml.findall('permissions/grant/*/domains/id')
    for domain_id_element in domain_id_elements:
        domain_id_element.text = domain_id

    try:
        permissions_xsd.assertValid(permissions_xml)
    except etree.DocumentInvalid as e:
        raise RuntimeError(str(e))

    with open(path, 'wb') as f:
        f.write(etree.tostring(permissions_xml, pretty_print=True))


def get_policy(name, policy_file_path):
    policy_tree = load_policy(policy_file_path)
    return get_policy_from_tree(name, policy_tree)


def get_policy_from_tree(name, policy_tree):
    context_element = policy_tree.find(
        path='contexts/context[@path="{path}"]'.format(path=name))
    if context_element is None:
        raise RuntimeError('unable to find context "{name}"'.format(
            name=name
        ))
    contexts_element = etree.Element('contexts')
    contexts_element.append(context_element)
    policy_element = etree.Element('policy')
    policy_element.append(contexts_element)
    return policy_element


def create_permission(keystore_path, identity, policy_file_path):
    policy_element = get_policy(identity, policy_file_path)
    create_permissions_from_policy_element(keystore_path, identity, policy_element)
    return True


def create_permissions_from_policy_element(keystore_path, identity, policy_element):
    domain_id = os.getenv(DOMAIN_ID_ENV, '0')
    relative_path = os.path.normpath(identity.lstrip('/'))
    key_dir = os.path.join(keystore_path, 'contexts', relative_path)
    print("creating permission file for identity: '%s'" % identity)
    permissions_path = os.path.join(key_dir, 'permissions.xml')
    create_permission_file(permissions_path, domain_id, policy_element)

    signed_permissions_path = os.path.join(key_dir, 'permissions.p7s')
    keystore_ca_cert_path = os.path.join(keystore_path, 'public', 'ca.cert.pem')
    keystore_ca_key_path = os.path.join(keystore_path, 'private', 'ca.key.pem')
    _create_smime_signed_file(
        keystore_ca_cert_path, keystore_ca_key_path, permissions_path, signed_permissions_path)


def create_key(keystore_path, identity):
    if not is_valid_keystore(keystore_path):
        print("'%s' is not a valid keystore " % keystore_path)
        return False
    if not is_key_name_valid(identity):
        return False
    print("creating key for identity: '%s'" % identity)

    relative_path = os.path.normpath(identity.lstrip('/'))
    key_dir = os.path.join(keystore_path, 'contexts', relative_path)
    os.makedirs(key_dir, exist_ok=True)

    # symlink the CA cert in there
    public_certs = ['identity_ca.cert.pem', 'permissions_ca.cert.pem']
    for public_cert in public_certs:
        dst = os.path.join(key_dir, public_cert)
        keystore_ca_cert_path = os.path.join(keystore_path, 'public', public_cert)
        relativepath = os.path.relpath(keystore_ca_cert_path, key_dir)
        try:
            os.symlink(src=relativepath, dst=dst)
        except FileExistsError as e:
            if not os.path.samefile(keystore_ca_cert_path, dst):
                print('Existing symlink does not match!')
                raise RuntimeError(str(e))

    # symlink the governance file in there
    keystore_governance_path = os.path.join(keystore_path, 'contexts', 'governance.p7s')
    dest_governance_path = os.path.join(key_dir, 'governance.p7s')
    relativepath = os.path.relpath(keystore_governance_path, key_dir)
    os.symlink(src=relativepath, dst=dest_governance_path)

    keystore_identity_ca_cert_path = os.path.join(keystore_path, 'public', 'identity_ca.cert.pem')
    keystore_identity_ca_key_path = os.path.join(keystore_path, 'private', 'identity_ca.key.pem')

    cert_path = os.path.join(key_dir, 'cert.pem')
    key_path = os.path.join(key_dir, 'key.pem')
    if not os.path.isfile(cert_path) or not os.path.isfile(key_path):
        print('creating cert and key')
        _create_key_and_cert(
            keystore_identity_ca_cert_path, keystore_identity_ca_key_path, identity, cert_path, key_path)
    else:
        print('found cert and key; not creating new ones!')

    # create a wildcard permissions file for this node which can be overridden
    # later using a policy if desired
    policy_file_path = get_policy_default('policy.xml')
    policy_element = get_policy('/', policy_file_path)
    context_element = policy_element.find('contexts/context')
    context_element.attrib['path'] = identity

    permissions_path = os.path.join(key_dir, 'permissions.xml')
    domain_id = os.getenv(DOMAIN_ID_ENV, '0')
    create_permission_file(permissions_path, domain_id, policy_element)

    signed_permissions_path = os.path.join(key_dir, 'permissions.p7s')
    keystore_permissions_ca_key_path = os.path.join(keystore_path, 'private', 'permissions_ca.key.pem')
    _create_smime_signed_file(
        keystore_ca_cert_path, keystore_permissions_ca_key_path, permissions_path, signed_permissions_path)

    return True


def list_keys(keystore_path):
    for name in os.listdir(keystore_path):
        if os.path.isdir(os.path.join(keystore_path, name)):
            print(name)
    return True


def distribute_key(source_keystore_path, taget_keystore_path):
    raise NotImplementedError()


def get_keystore_path_from_env():
    root_keystore_env_var = 'ROS_SECURITY_ROOT_DIRECTORY'
    root_keystore_path = os.getenv(root_keystore_env_var)
    if root_keystore_path is None:
        print('%s is empty' % root_keystore_env_var, file=sys.stderr)
    return root_keystore_path


def generate_artifacts(keystore_path=None, identity_names=[], policy_files=[]):
    if keystore_path is None:
        keystore_path = get_keystore_path_from_env()
        if keystore_path is None:
            return False
    if not is_valid_keystore(keystore_path):
        print('%s is not a valid keystore, creating new keystore' % keystore_path)
        create_keystore(keystore_path)

    # create keys for all provided identities
    for identity in identity_names:
        if not create_key(keystore_path, identity):
            return False
    for policy_file in policy_files:
        policy_tree = load_policy(policy_file)
        contexts_element = policy_tree.find('contexts')
        for context in contexts_element:
            identity_name = context.get('path')
            if identity_name not in identity_names:
                if not create_key(keystore_path, identity_name):
                    return False
            policy_element = get_policy_from_tree(identity_name, policy_tree)
            create_permissions_from_policy_element(
                keystore_path, identity_name, policy_element)
    return True


def _sign_bytes(cert, key, byte_string):
    # Using two flags here to get the output required:
    #   - PKCS7_DETACHED: Use cleartext signing
    #   - PKCS7_TEXT: Set the MIME headers for text/plain
    flags = SSLBinding.lib.PKCS7_DETACHED
    flags |= SSLBinding.lib.PKCS7_TEXT

    # Convert the byte string into a buffer for SSL
    bio_in = SSLBinding.lib.BIO_new_mem_buf(byte_string, len(byte_string))
    try:
        pkcs7 = SSLBinding.lib.PKCS7_sign(
            cert._x509, key._evp_pkey, SSLBinding.ffi.NULL, bio_in, flags)
    finally:
        # Free the memory allocated for the buffer
        SSLBinding.lib.BIO_free(bio_in)

    # PKCS7_sign consumes the buffer; allocate a new one again to get it into the final document
    bio_in = SSLBinding.lib.BIO_new_mem_buf(byte_string, len(byte_string))
    try:
        # Allocate a buffer for the output document
        bio_out = SSLBinding.lib.BIO_new(SSLBinding.lib.BIO_s_mem())
        try:
            # Write the final document out to the buffer
            SSLBinding.lib.SMIME_write_PKCS7(bio_out, pkcs7, bio_in, flags)

            # Copy the output document back to python-managed memory
            result_buffer = SSLBinding.ffi.new('char**')
            buffer_length = SSLBinding.lib.BIO_get_mem_data(bio_out, result_buffer)
            output = SSLBinding.ffi.buffer(result_buffer[0], buffer_length)[:]
        finally:
            # Free the memory required for the output buffer
            SSLBinding.lib.BIO_free(bio_out)
    finally:
        # Free the memory allocated for the input buffer
        SSLBinding.lib.BIO_free(bio_in)

    return output


def _create_smime_signed_file(cert_path, key_path, unsigned_file_path, signed_file_path):
    # Load the CA cert and key from disk
    with open(cert_path, 'rb') as cert_file:
        cert = x509.load_pem_x509_certificate(
            cert_file.read(), cryptography_backend())

    with open(key_path, 'rb') as key_file:
        private_key = serialization.load_pem_private_key(
            key_file.read(), None, cryptography_backend())

    # Get the contents of the unsigned file, which we're about to sign
    with open(unsigned_file_path, 'rb') as f:
        content = f.read()

    # Sign the contents, and write the result to the appropriate place
    with open(signed_file_path, 'wb') as f:
        f.write(_sign_bytes(cert, private_key, content))


def _build_key_and_cert(subject_name, *, ca=False, ca_key=None, issuer_name=''):
    if not issuer_name:
        issuer_name = subject_name

    # DDS-Security section 9.3.1 calls for prime256v1, for which SECP256R1 is an alias
    private_key = ec.generate_private_key(ec.SECP256R1, cryptography_backend())
    if not ca_key:
        ca_key = private_key

    if ca:
        extension = x509.BasicConstraints(ca=True, path_length=1)
    else:
        extension = x509.BasicConstraints(ca=False, path_length=None)

    utcnow = datetime.datetime.utcnow()
    builder = x509.CertificateBuilder(
        ).issuer_name(
            issuer_name
        ).serial_number(
            x509.random_serial_number()
        ).not_valid_before(
            utcnow
        ).not_valid_after(
            # TODO: This should not be hard-coded
            utcnow + datetime.timedelta(days=3650)
        ).public_key(
            private_key.public_key()
        ).subject_name(
            subject_name
        ).add_extension(
            extension, critical=ca
        )
    cert = builder.sign(ca_key, hashes.SHA256(), cryptography_backend())

    return (cert, private_key)


def _create_key_and_cert(
        keystore_ca_cert_path, keystore_ca_key_path, identity, cert_path, key_path):
    # Load the CA cert and key from disk
    with open(keystore_ca_cert_path, 'rb') as f:
        ca_cert = x509.load_pem_x509_certificate(f.read(), cryptography_backend())

    with open(keystore_ca_key_path, 'rb') as f:
        ca_key = serialization.load_pem_private_key(f.read(), None, cryptography_backend())

    cert, private_key = _build_key_and_cert(
        x509.Name([x509.NameAttribute(x509.oid.NameOID.COMMON_NAME, identity)]),
        issuer_name=ca_cert.subject,
        ca_key=ca_key)

    _write_key(private_key, key_path)
    _write_cert(cert, cert_path)
