#! /usr/bin/env python3

import argparse
import os
import pandas as pd
import sqlite3
import sys

from lxml import etree

from sros2.policy import (
    get_policy_schema,
    get_transport_schema,
    get_transport_template,
    POLICY_VERSION,
)

node_query = """
SELECT DISTINCT
    DCPSParticipant.ParticipantData_key,
    DCPSParticipant.ParticipantData_user_data
FROM DCPSParticipant
WHERE DCPSParticipant.ParticipantData_user_data IS NOT NULL;
"""

_query = """
SELECT DISTINCT
    DCPSParticipant.ParticipantData_key,
    DCPSParticipant.ParticipantData_user_data,
    DCPS{mode}.{mode}Data_topic_name
FROM DCPSParticipant 
INNER JOIN DCPS{mode} ON
    DCPSParticipant.ParticipantData_key = DCPS{mode}.{mode}Data_participant_key
AND DCPSParticipant.ParticipantData_user_data IS NOT NULL;
"""

node_pub_query = _query.format(mode="Publication")
node_sub_query = _query.format(mode="Subscription")


def user_bytes_to_dict(user_bytes):
    try:
        user_string = user_bytes.decode('utf8')
        key_value_list = user_string[:-2].split(';')
        key_value_dict = dict()
        for key_values in key_value_list:
            key, value = key_values.split('=', 1)
            key_value_dict[key] = value
        return key_value_dict
    except:
        return None


def translate_df(df):
    _df = df['ParticipantData_user_data'] \
        .apply(user_bytes_to_dict) \
        .apply(pd.Series)
    df = pd.concat([df, _df], axis=1)
    return df


def db_to_df(db):
    node_df = translate_df(pd.read_sql_query(node_query, db))
    pub_df = translate_df(pd.read_sql_query(node_pub_query, db))
    sub_df = translate_df(pd.read_sql_query(node_sub_query, db))

    pub_df = pub_df.assign(mode='publish')
    pub_df = pub_df.rename(columns={"PublicationData_topic_name": "dds_topic"})
    sub_df = sub_df.assign(mode='subscribe')
    sub_df = sub_df.rename(columns={"SubscriptionData_topic_name": "dds_topic"})

    df = pd.concat([node_df, pub_df, sub_df])
    df = df[df['namespace'].notnull()]
    df.set_index(['namespace', 'name', 'mode'], inplace=True)
    return df


def df_to_dds_policy(df):
    dds_policy = etree.Element('policy')
    dds_policy.set("version", POLICY_VERSION,)
    profiles = etree.SubElement(dds_policy, 'profiles')

    for namespace in df.index.get_level_values('namespace').unique():
        _df = df.loc[namespace, :, :]
        for name in _df.index.get_level_values('name').unique():
            profile = etree.SubElement(profiles, 'profile')
            profile.set("ns", namespace)
            profile.set("node", name)
            __df = df.loc[namespace, name, :]
            for mode in __df.index.get_level_values('mode').unique():
                if not pd.isna(mode):
                    topics = etree.SubElement(profile, 'dds_topics')
                    topics.set(mode, "ALLOW")
                    for dds_topic in df['dds_topic'].loc[namespace, name, mode]:
                        topic = etree.SubElement(topics, 'dds_topic')
                        topic.text = dds_topic
    return dds_policy


def dds_policy_to_sros2_policy(dds_policy):

    # Parse files
    policy_xsd = etree.XMLSchema(
        etree.parse(
            get_policy_schema('policy.xsd')))
    demangle_xsl = etree.XSLT(
        etree.parse(
            get_transport_template('dds', 'demangle.xsl')))

    # TODO: update schema for dds_topics?
    # Validate policy schema
    # policy_xsd.assertValid(dds_policy)

    # Transform policy
    sros2_policy = demangle_xsl(dds_policy)

    # Validate policy schema
    policy_xsd.assertValid(sros2_policy)

    return sros2_policy


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-i', '--input-db', required=True,
        help='path to SQLite3 database with discovery data')
    parser.add_argument(
        '-o', '--output-policy',
        help='path to XML policy file with generated output')
    args = parser.parse_args(argv)

    db_uri = 'file:{}?mode=ro'.format(args.input_db)
    with sqlite3.connect(db_uri, uri=True) as db:
        df = db_to_df(db)
    
    dds_policy = df_to_dds_policy(df)
    sros2_policy = dds_policy_to_sros2_policy(dds_policy)

    if args.output_policy is not None:
        with open(args.output_policy, 'wb') as f:
            f.write(etree.tostring(sros2_policy, pretty_print=True))
    else:
        print(etree.tostring(sros2_policy, pretty_print=True).decode())


if __name__ == '__main__':
    main()