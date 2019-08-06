
# coding: utf-8

# In[ ]:


import pandas as pd
import sqlite3

from lxml import etree


# In[ ]:


db_path = "../tb3_demo_dds_discovery.db"


# In[ ]:


node_pub_query = """
SELECT DISTINCT
    DCPSParticipant.ParticipantData_key,
    DCPSParticipant.ParticipantData_user_data,
    DCPSPublication.PublicationData_topic_name
FROM DCPSParticipant 
INNER JOIN DCPSPublication ON
    DCPSParticipant.ParticipantData_key = DCPSPublication.PublicationData_participant_key
AND DCPSParticipant.ParticipantData_user_data IS NOT NULL;
"""

node_sub_query = """
SELECT DISTINCT
    DCPSParticipant.ParticipantData_key,
    DCPSParticipant.ParticipantData_user_data,
    DCPSSubscription.SubscriptionData_topic_name
FROM DCPSParticipant 
INNER JOIN DCPSSubscription ON
    DCPSParticipant.ParticipantData_key=DCPSSubscription.SubscriptionData_participant_key
AND DCPSParticipant.ParticipantData_user_data IS NOT NULL;
"""


# In[ ]:


def user_bytes_to_dict(user_bytes):
    user_string = user_bytes.decode('utf8')
    key_value_list = user_string[:-2].split(';')
    key_value_dict = dict()
    for key_values in key_value_list:
        key, value = key_values.split('=', 1)
        key_value_dict[key] = value
    return key_value_dict


# In[ ]:


def translate_df(df):
    _df = df['ParticipantData_user_data'].apply(user_bytes_to_dict).apply(pd.Series)
    df = pd.concat([df, _df], axis=1)
    return df


# In[ ]:


db_uri = 'file:{}?mode=ro'.format(db_path)
with sqlite3.connect(db_uri, uri=True) as db:
    pub_df = translate_df(pd.read_sql_query(node_pub_query, db))
    sub_df = translate_df(pd.read_sql_query(node_sub_query, db))

pub_df = pub_df.assign(mode='publish')
pub_df = pub_df.rename(columns={"PublicationData_topic_name": "dds_topic"})
sub_df = sub_df.assign(mode='subscribe')
sub_df = sub_df.rename(columns={"SubscriptionData_topic_name": "dds_topic"})

df = pd.concat([pub_df, sub_df])
df.set_index(['namespace', 'name', 'mode'], inplace=True)


# In[ ]:


foo = df.index[0]
foo


# In[ ]:


# with pd.option_context('display.max_rows', None, 'display.max_columns', None):  # more options can be specified also
#     display(df)


# In[ ]:


polciy = etree.Element('polciy')
profiles = etree.SubElement(polciy, 'profiles')

for namespace in df.index.get_level_values('namespace').unique():
    for name in df.index.get_level_values('name').unique():
        profile = etree.SubElement(profiles, 'profile')
        profile.set("ns", namespace)
        profile.set("node", name)
        _df = df['dds_topic'].loc[namespace, name, :]
        for mode in _df.index.get_level_values('mode').unique():
            topics = etree.SubElement(profile, 'dds_topics')
            topics.set(mode, "ALLOW")
            for dds_topic in df['dds_topic'].loc[namespace, name, :]:
                topic = etree.SubElement(topics, 'dds_topic')
                topic.text = dds_topic


# In[ ]:


print(etree.tostring(polciy, pretty_print=True).decode())


# In[ ]:


import os

from lxml import etree

from sros2.policy import (
    get_policy_schema,
    get_transport_schema,
    get_transport_template,
)


# In[ ]:


# Get paths
policy_xsd_path = get_policy_schema('policy.xsd')
permissions_xsl_path = get_transport_template('dds', 'permissions.xsl')
permissions_xsd_path = get_transport_schema('dds', 'permissions.xsd')
policy_xsl_path = 'sros2/sros2/policy/templates/dds/policy.xsl'

# Parse files
policy_xsd = etree.XMLSchema(etree.parse(policy_xsd_path))
policy_xsl = etree.XSLT(etree.parse(policy_xsl_path))
permissions_xsl = etree.XSLT(etree.parse(permissions_xsl_path))
permissions_xsd = etree.XMLSchema(etree.parse(permissions_xsd_path))

# Get policy
dds_policy_xml_path = 'tb3_raw_dds_policy.xml'
dds_policy_xml = etree.parse(dds_policy_xml_path)
dds_policy_xml.xinclude()

# Validate policy schema
# policy_xsd.assertValid(policy_xml)

# Transform policy
policy_xml = policy_xsl(dds_policy_xml)

# Validate permissions schema
# policy_xsd.assertValid(policy_xml)

# Output policy
# policy_xml_path = os.path.join('policy.xml')
# with open(policy_xml_path, 'w') as f:
#     f.write(etree.tostring(policy_xml, pretty_print=True).decode())
print(etree.tostring(policy_xml, pretty_print=True).decode())

