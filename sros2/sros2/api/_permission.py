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

from lxml import etree

from rclpy.utilities import get_rmw_implementation_identifier

from sros2.policy import get_transport_schema, get_transport_template

from . import _keystore, _policy, _utilities


_RMW_WITH_ROS_GRAPH_INFO_TOPIC = ('rmw_fastrtps_cpp', 'rmw_fastrtps_dynamic_cpp')


def create_permission(keystore_path, identity, policy_file_path):
    policy_element = _policy.get_policy(identity, policy_file_path)
    create_permissions_from_policy_element(keystore_path, identity, policy_element)
    return True


def create_permissions_from_policy_element(keystore_path, identity, policy_element):
    relative_path = os.path.normpath(identity.lstrip('/'))
    key_dir = os.path.join(_keystore.get_keystore_enclaves_dir(keystore_path), relative_path)
    print("creating permission file for identity: '%s'" % identity)
    permissions_path = os.path.join(key_dir, 'permissions.xml')
    create_permission_file(permissions_path, _utilities.domain_id(), policy_element)

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


def create_permission_file(path, domain_id, policy_element):
    print('creating permission')
    permissions_xsl_path = get_transport_template('dds', 'permissions.xsl')
    permissions_xsl = etree.XSLT(etree.parse(permissions_xsl_path))
    permissions_xsd_path = get_transport_schema('dds', 'permissions.xsd')
    permissions_xsd = etree.XMLSchema(etree.parse(permissions_xsd_path))

    kwargs = {}

    cert_path = os.path.join(os.path.dirname(path), 'cert.pem')
    cert_content = _utilities.load_cert(cert_path)
    kwargs['not_valid_before'] = etree.XSLT.strparam(cert_content.not_valid_before.isoformat())
    kwargs['not_valid_after'] = etree.XSLT.strparam(cert_content.not_valid_after.isoformat())

    if get_rmw_implementation_identifier() in _RMW_WITH_ROS_GRAPH_INFO_TOPIC:
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
