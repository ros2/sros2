from lxml import etree

from sros2.policy import (
    get_policy_schema,
    get_transport_schema,
    get_transport_template,
)

# Get paths
policy_xml_path = 'sample_policy.xml'
policy_xsd_path = get_policy_schema('policy.xsd')
policy_xsl_path = get_transport_template('dds', 'policy.xsl')
permissions_xsl_path = get_transport_template('dds', 'permissions.xsl')
permissions_xsd_path = get_transport_schema('dds', 'permissions.xsd')

# Parse files
policy_xml = etree.parse(policy_xml_path)
policy_xsd = etree.XMLSchema(etree.parse(policy_xsd_path))
policy_xsl = etree.XSLT(etree.parse(policy_xsl_path))
permissions_xsl = etree.XSLT(etree.parse(permissions_xsl_path))
permissions_xsd = etree.XMLSchema(etree.parse(permissions_xsd_path))

# Validate policy schema
policy_xsd.assertValid(policy_xml)

# Transform policy
_permissions_xml = policy_xsl(policy_xml)
permissions_xml = permissions_xsl(_permissions_xml)

# Validate permissions schema
permissions_xsd.assertValid(permissions_xml)

# Output permissions
print(etree.tostring(permissions_xml, pretty_print=True).decode())
