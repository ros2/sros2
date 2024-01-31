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

from collections import namedtuple
import pathlib
import sys

from argcomplete.completers import FilesCompleter

from lxml import etree

from ros2cli.node.strategy import add_arguments as add_strategy_node_arguments
from ros2cli.node.strategy import NodeStrategy

from sros2.policy import (
    dump_policy,
    load_policy,
    POLICY_VERSION,
)

from sros2.verb import VerbExtension


_HIDDEN_NODE_PREFIX = '_'

_NodeName = namedtuple('_NodeName', ('node', 'ns', 'fqn', 'path'))
_TopicInfo = namedtuple('_TopicInfo', ('fqn', 'type'))


class GeneratePolicyVerb(VerbExtension):
    """Generate XML policy file from ROS graph data."""

    def add_arguments(self, parser, cli_name) -> None:
        arg = parser.add_argument(
            'POLICY_FILE_PATH', type=pathlib.Path, help='path of the policy xml file')
        arg.completer = FilesCompleter(
            allowednames=('xml'), directories=False)
        add_strategy_node_arguments(parser)

    def get_policy(self, policy_file_path: pathlib.Path):
        if policy_file_path.is_file():
            return load_policy(policy_file_path)
        else:
            enclaves = etree.Element('enclaves')
            policy = etree.Element('policy')
            policy.attrib['version'] = POLICY_VERSION
            policy.append(enclaves)
            return policy

    def get_profile(self, policy, node_name):
        enclave = policy.find(
            path=f'enclaves/enclave[@path="{node_name.path}"]')
        if enclave is None:
            enclave = etree.Element('enclave')
            enclave.attrib['path'] = node_name.path
            profiles = etree.Element('profiles')
            enclave.append(profiles)
            enclaves = policy.find('enclaves')
            enclaves.append(enclave)
        profile = enclave.find(
            path=f'profiles/profile[@ns="{node_name.ns}"][@node="{node_name.node}"]')
        if profile is None:
            profile = etree.Element('profile')
            profile.attrib['ns'] = node_name.ns
            profile.attrib['node'] = node_name.node
            profiles = enclave.find('profiles')
            profiles.append(profile)
        return profile

    def get_permissions(self, profile, permission_type, rule_type, rule_qualifier):
        permissions = profile.find(
            path=f'{permission_type}s[@{rule_type}="{rule_qualifier}"]')
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

    def main(self, *, args) -> int:
        policy = self.get_policy(args.POLICY_FILE_PATH)
        with NodeStrategy(args) as node:
            node_names = _get_node_names(node=node, include_hidden_nodes=False)

            if not len(node_names):
                print('No nodes detected in the ROS graph. No policy file was generated.',
                      file=sys.stderr)
                return 1

            for node_name in node_names:
                profile = self.get_profile(policy, node_name)
                subscribe_topics = _get_subscriber_info(node=node, node_name=node_name)
                if subscribe_topics:
                    self.add_permission(
                        profile, 'topic', 'subscribe', 'ALLOW', subscribe_topics, node_name)
                publish_topics = _get_publisher_info(node=node, node_name=node_name)
                if publish_topics:
                    self.add_permission(
                        profile, 'topic', 'publish', 'ALLOW', publish_topics, node_name)
                reply_services = _get_service_info(node=node, node_name=node_name)
                if reply_services:
                    self.add_permission(
                        profile, 'service', 'reply', 'ALLOW', reply_services, node_name)
                request_services = _get_client_info(node=node, node_name=node_name)
                if request_services:
                    self.add_permission(
                        profile, 'service', 'request', 'ALLOW', request_services, node_name)

        with open(args.POLICY_FILE_PATH, 'w') as stream:
            dump_policy(policy, stream)
        return 0


def _get_node_names(*, node, include_hidden_nodes=False):
    node_names_and_namespaces_with_enclaves = node.get_node_names_and_namespaces_with_enclaves()
    return [
        _NodeName(
            node=t[0],
            ns=t[1],
            fqn=t[1] + ('' if t[1].endswith('/') else '/') + t[0],
            path=t[2])
        for t in node_names_and_namespaces_with_enclaves
        if (
            include_hidden_nodes or
            (t[0] and not t[0].startswith(_HIDDEN_NODE_PREFIX))
        )
    ]


def _get_topics(node_name, func):
    names_and_types = func(node_name.node, node_name.ns)
    return [
        _TopicInfo(
            fqn=t[0],
            type=t[1])
        for t in names_and_types]


def _get_subscriber_info(node, node_name):
    return _get_topics(node_name, node.get_subscriber_names_and_types_by_node)


def _get_publisher_info(node, node_name):
    return _get_topics(node_name, node.get_publisher_names_and_types_by_node)


def _get_service_info(node, node_name):
    return _get_topics(node_name, node.get_service_names_and_types_by_node)


def _get_client_info(node, node_name):
    return _get_topics(node_name, node.get_client_names_and_types_by_node)
