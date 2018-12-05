import lxml.etree as ET

# Set paths
profile_xml_path = 'sample_policy.xml'
profile_xslt_path = 'profile_template.xslt'
permissions_xslt_path = 'permissions_template.xslt'

# Load profile
profile_xml = ET.parse(profile_xml_path)

# Load templates
profile_xslt = ET.XSLT(ET.parse(profile_xslt_path))
permissions_xslt = ET.XSLT(ET.parse(permissions_xslt_path))

# Apply permission transform
permissions_xml = profile_xslt(profile_xml)
permissions_xml = permissions_xslt(permissions_xml)

# Show transform
print(ET.tostring(permissions_xml, pretty_print=True).decode())
