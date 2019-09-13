# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import ament_index_python
from lxml import etree

from sros2.policy import (
    get_policy_schema,
    get_transport_schema,
    get_transport_template,
)


if 'XML_CATALOG_FILES' not in os.environ:
    os.environ['XML_CATALOG_FILES'] = os.path.join(
        ament_index_python.get_package_share_directory('sros2'),
        'xml_cache',
        'xhtml-cache.xml'
    )
elif 'xml_cache/xhtml-cache.xml' not in os.environ['XML_CATALOG_FILES']:
    os.environ['XML_CATALOG_FILES'] += ' ' + os.path.join(
        ament_index_python.get_package_share_directory('sros2'),
        'xml_cache',
        'xhtml-cache.xml'
    )


def test_policy_to_permissions():
    # Get paths
    policy_xsd_path = get_policy_schema('policy.xsd')
    permissions_xsl_path = get_transport_template('dds', 'permissions.xsl')
    permissions_xsd_path = get_transport_schema('dds', 'permissions.xsd')

    # Parse files
    policy_xsd = etree.XMLSchema(etree.parse(policy_xsd_path))
    permissions_xsl = etree.XSLT(etree.parse(permissions_xsl_path))
    permissions_xsd = etree.XMLSchema(etree.parse(permissions_xsd_path))

    # Get policy
    test_dir = os.path.dirname(os.path.abspath(__file__))
    policy_xml_path = os.path.join(test_dir, 'policies', 'sample_policy.xml')
    policy_xml = etree.parse(policy_xml_path)
    policy_xml.xinclude()

    # Validate policy schema
    policy_xsd.assertValid(policy_xml)

    # Transform policy
    permissions_xml = permissions_xsl(policy_xml)

    # Validate permissions schema
    permissions_xsd.assertValid(permissions_xml)

    # Assert expected permissions
    permissions_xml_path = os.path.join(test_dir, 'policies', 'permissions.xml')
    with open(permissions_xml_path) as f:
        expected = f.read()
        actual = etree.tostring(permissions_xml, pretty_print=True).decode()
        assert actual == expected
