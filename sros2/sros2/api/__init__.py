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
import errno
import os
import sys

from cryptography import x509
from cryptography.hazmat.backends import default_backend as cryptography_backend
from cryptography.hazmat.primitives import serialization

from lxml import etree

from rclpy.exceptions import InvalidNamespaceException
from rclpy.utilities import get_rmw_implementation_identifier
from rclpy.validate_namespace import validate_namespace

from sros2.policy import (
    get_policy_default,
    get_transport_schema,
    get_transport_template,
    load_policy,
)

from . import _keystore, _utilities

HIDDEN_NODE_PREFIX = '_'

NodeName = namedtuple('NodeName', ('node', 'ns', 'fqn'))
TopicInfo = namedtuple('Topic', ('fqn', 'type'))

RMW_WITH_ROS_GRAPH_INFO_TOPIC = ('rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp')


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


def is_key_name_valid(name):
    # TODO(ivanpauno): Use validate_security_context_name when it's propagated to `rclpy`.
    #   This is not to bad for the moment.
    #   Related with https://github.com/ros2/rclpy/issues/528.
    try:
        return validate_namespace(name)
    except InvalidNamespaceException as e:
        print(e)
        return False


def create_permission_file(path, domain_id, policy_element):
    print('creating permission')
    permissions_xsl_path = get_transport_template('dds', 'permissions.xsl')
    permissions_xsl = etree.XSLT(etree.parse(permissions_xsl_path))
    permissions_xsd_path = get_transport_schema('dds', 'permissions.xsd')
    permissions_xsd = etree.XMLSchema(etree.parse(permissions_xsd_path))

    kwargs = {}
    if get_rmw_implementation_identifier() in RMW_WITH_ROS_GRAPH_INFO_TOPIC:
        kwargs['allow_ros_discovery_topic'] = etree.XSLT.strparam('1')
    permissions_xml = permissions_xsl(policy_element, **kwargs)

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
        path=f'contexts/context[@path="{name}"]')
    if context_element is None:
        raise RuntimeError(f'unable to find context "{name}"')
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
    relative_path = os.path.normpath(identity.lstrip('/'))
    key_dir = os.path.join(_keystore.keystore_context_dir(keystore_path), relative_path)
    print("creating permission file for identity: '%s'" % identity)
    permissions_path = os.path.join(key_dir, 'permissions.xml')
    create_permission_file(permissions_path, _utilities.domain_id(), policy_element)

    signed_permissions_path = os.path.join(key_dir, 'permissions.p7s')
    keystore_ca_cert_path = os.path.join(
        _keystore.keystore_public_dir(keystore_path), 'ca.cert.pem')
    keystore_ca_key_path = os.path.join(
        _keystore.keystore_private_dir(keystore_path), 'ca.key.pem')
    _utilities.create_smime_signed_file(
        keystore_ca_cert_path, keystore_ca_key_path, permissions_path, signed_permissions_path)


def create_key(keystore_path, identity):
    if not _keystore.is_valid_keystore(keystore_path):
        print("'%s' is not a valid keystore " % keystore_path)
        return False
    if not is_key_name_valid(identity):
        return False
    print("creating key for identity: '%s'" % identity)

    relative_path = os.path.normpath(identity.lstrip('/'))
    key_dir = os.path.join(_keystore.keystore_context_dir(keystore_path), relative_path)
    os.makedirs(key_dir, exist_ok=True)

    # symlink the CA cert in there
    public_certs = ['identity_ca.cert.pem', 'permissions_ca.cert.pem']
    for public_cert in public_certs:
        dst = os.path.join(key_dir, public_cert)
        keystore_ca_cert_path = os.path.join(
            _keystore.keystore_public_dir(keystore_path), public_cert)
        relativepath = os.path.relpath(keystore_ca_cert_path, key_dir)
        _utilities.create_symlink(src=relativepath, dst=dst)

    # symlink the governance file in there
    keystore_governance_path = os.path.join(
        _keystore.keystore_context_dir(keystore_path), 'governance.p7s')
    dest_governance_path = os.path.join(key_dir, 'governance.p7s')
    relativepath = os.path.relpath(keystore_governance_path, key_dir)
    _utilities.create_symlink(src=relativepath, dst=dest_governance_path)

    keystore_identity_ca_cert_path = os.path.join(
        _keystore.keystore_public_dir(keystore_path), 'identity_ca.cert.pem')
    keystore_identity_ca_key_path = os.path.join(
        _keystore.keystore_private_dir(keystore_path), 'identity_ca.key.pem')

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
    policy_element = get_policy('/', policy_file_path)
    context_element = policy_element.find('contexts/context')
    context_element.attrib['path'] = identity

    permissions_path = os.path.join(key_dir, 'permissions.xml')
    create_permission_file(permissions_path, _utilities.domain_id(), policy_element)

    signed_permissions_path = os.path.join(key_dir, 'permissions.p7s')
    keystore_permissions_ca_key_path = os.path.join(
        _keystore.keystore_private_dir(keystore_path), 'permissions_ca.key.pem')
    _utilities.create_smime_signed_file(
        keystore_ca_cert_path,
        keystore_permissions_ca_key_path,
        permissions_path,
        signed_permissions_path
    )

    return True


def list_keys(keystore_path):
    contexts_path = _keystore.keystore_context_dir(keystore_path)
    if not os.path.isdir(keystore_path):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), keystore_path)
    if not os.path.isdir(contexts_path):
        return True
    for name in os.listdir(contexts_path):
        if os.path.isdir(os.path.join(contexts_path, name)):
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
    if not _keystore.is_valid_keystore(keystore_path):
        print('%s is not a valid keystore, creating new keystore' % keystore_path)
        _keystore.create_keystore(keystore_path)

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


def _create_key_and_cert(
        keystore_ca_cert_path, keystore_ca_key_path, identity, cert_path, key_path):
    # Load the CA cert and key from disk
    with open(keystore_ca_cert_path, 'rb') as f:
        ca_cert = x509.load_pem_x509_certificate(f.read(), cryptography_backend())

    with open(keystore_ca_key_path, 'rb') as f:
        ca_key = serialization.load_pem_private_key(f.read(), None, cryptography_backend())

    cert, private_key = _utilities.build_key_and_cert(
        x509.Name([x509.NameAttribute(x509.oid.NameOID.COMMON_NAME, identity)]),
        issuer_name=ca_cert.subject,
        ca_key=ca_key)

    _utilities.write_key(private_key, key_path)
    _utilities.write_cert(cert, cert_path)
