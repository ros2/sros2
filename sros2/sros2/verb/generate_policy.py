# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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
import sys

try:
    from argcomplete.completers import DirectoriesCompleter
except ImportError:
    def DirectoriesCompleter():
        return None
try:
    from argcomplete.completers import FilesCompleter
except ImportError:
    def FilesCompleter(*, allowednames, directories):
        return None

from lxml import etree

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import NodeStrategy

from sros2.api import (
    get_client_info,
    get_node_names,
    get_publisher_info,
    get_service_info,
    get_subscriber_info
)

from sros2.policy import (
    dump_policy,
    load_policy,
    POLICY_VERSION,
)

from sros2.verb import VerbExtension


def formatTopics(topic_list, permission, topic_map):
    for topic in topic_list:
        topic_map[topic.name].append(permission)


class GeneratePolicyVerb(VerbExtension):
    """Generate XML policy file from ROS graph data."""

    def add_arguments(self, parser, cli_name):

        arg = parser.add_argument(
            'POLICY_FILE_PATH', help='path of the policy xml file')
        arg.completer = FilesCompleter(
            allowednames=('xml'), directories=False)

    def get_policy(self, policy_file_path):
        if os.path.isfile(policy_file_path):
            return load_policy(policy_file_path)
        else:
            profiles = etree.Element('profiles')
            policy = etree.Element('policy')
            policy.attrib['version'] = POLICY_VERSION
            policy.append(profiles)
            return policy

    def get_profile(self, policy, node_name):
        profile = policy.find(
            path='profiles/profile[@ns="{ns}"][@node="{node}"]'.format(
                ns=node_name.ns,
                node=node_name.node))
        if profile is None:
            profile = etree.Element('profile')
            profile.attrib['ns'] = node_name.ns
            profile.attrib['node'] = node_name.node
            profiles = policy.find('profiles')
            profiles.append(profile)
        return profile

    def get_permissions(self, profile, permission_type, rule_type, rule_qualifier):
        permissions = profile.find(
            path='{permission_type}s[@{rule_type}="{rule_qualifier}"]'.format(
                permission_type=permission_type,
                rule_type=rule_type,
                rule_qualifier=rule_qualifier))
        if permissions is None:
            permissions = etree.Element(permission_type + 's')
            permissions.attrib[rule_type] = rule_qualifier
            profile.append(permissions)
        return permissions

    def add_permission(
            self, profile, permission_type, rule_type, rule_qualifier, expressions, node_name):
        permissions = self.get_permissions(profile, permission_type, rule_type, rule_qualifier)
        for expression in expressions:
            permission = etree.Element(permission_type)
            if expression.fqn.startswith(node_name.fqn + '/'):
                permission.text = '~' + expression.fqn[len(node_name.fqn):]
            elif expression.fqn.startswith(node_name.ns + '/'):
                permission.text = expression.fqn[len(node_name.ns + '/'):]
            elif expression.fqn.count('/') == 1 and node_name.ns == '/':
                permission.text = expression.fqn[len('/'):]
            else:
                permission.text = expression.fqn
            permissions.append(permission)

    def main(self, *, args):
        policy = self.get_policy(args.POLICY_FILE_PATH)
        node_names = []
        with NodeStrategy(args) as node:
            node_names = get_node_names(node=node, include_hidden_nodes=False)

        if not len(node_names):
            print('No nodes detected in the ROS graph. No policy file was generated.',
                  file=sys.stderr)
            return 1

        with DirectNode(args) as node:
            for node_name in node_names:
                profile = self.get_profile(policy, node_name)
                subscribe_topics = get_subscriber_info(node=node, node_name=node_name)
                if subscribe_topics:
                    self.add_permission(
                        profile, 'topic', 'subscribe', 'ALLOW', subscribe_topics, node_name)
                publish_topics = get_publisher_info(node=node, node_name=node_name)
                if publish_topics:
                    self.add_permission(
                        profile, 'topic', 'publish', 'ALLOW', publish_topics, node_name)
                reply_services = get_service_info(node=node, node_name=node_name)
                if reply_services:
                    self.add_permission(
                        profile, 'service', 'reply', 'ALLOW', reply_services, node_name)
                request_services = get_client_info(node=node, node_name=node_name)
                if request_services:
                    self.add_permission(
                        profile, 'service', 'request', 'ALLOW', request_services, node_name)

        with open(args.POLICY_FILE_PATH, 'w') as stream:
            dump_policy(policy, stream)
        return 0
