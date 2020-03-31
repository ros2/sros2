# Copyright 2018-2019 Open Source Robotics Foundation, Inc.
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

import pkg_resources

POLICY_VERSION = '0.2.0'


def get_policy_default(name):
    return pkg_resources.resource_filename(
        package_or_requirement='sros2',
        resource_name=os.path.join('policy', 'defaults', name))


def get_policy_schema(name):
    return pkg_resources.resource_filename(
        package_or_requirement='sros2',
        resource_name=os.path.join('policy', 'schemas', name))


def get_policy_template(name):
    return pkg_resources.resource_filename(
        package_or_requirement='sros2',
        resource_name=os.path.join('policy', 'templates', name))


def get_transport_default(transport, name):
    return pkg_resources.resource_filename(
        package_or_requirement='sros2',
        resource_name=os.path.join('policy', 'defaults', transport, name))


def get_transport_schema(transport, name):
    return pkg_resources.resource_filename(
        package_or_requirement='sros2',
        resource_name=os.path.join('policy', 'schemas', transport, name))


def get_transport_template(transport, name):
    return pkg_resources.resource_filename(
        package_or_requirement='sros2',
        resource_name=os.path.join('policy', 'templates', transport, name))


def load_policy(policy_file_path):
    if not os.path.isfile(policy_file_path):
        raise FileNotFoundError("policy file '%s' does not exist" % policy_file_path)
    policy = etree.parse(policy_file_path)
    policy.xinclude()
    try:
        policy_xsd_path = get_policy_schema('policy.xsd')
        policy_xsd = etree.XMLSchema(etree.parse(policy_xsd_path))
        policy_xsd.assertValid(policy)
    except etree.DocumentInvalid as e:
        raise RuntimeError(str(e))
    return policy


def dump_policy(policy, stream):
    policy_xsl_path = get_policy_template('policy.xsl')
    policy_xsl = etree.XSLT(etree.parse(policy_xsl_path))
    policy = policy_xsl(policy)
    try:
        policy_xsd_path = get_policy_schema('policy.xsd')
        policy_xsd = etree.XMLSchema(etree.parse(policy_xsd_path))
        policy_xsd.assertValid(policy)
    except etree.DocumentInvalid as e:
        raise RuntimeError(str(e))
    stream.write(etree.tostring(policy, pretty_print=True).decode())
