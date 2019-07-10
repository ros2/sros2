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

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import NodeStrategy

from sros2.api import (
    get_client_info,
    get_node_names,
    get_publisher_info,
    get_service_info,
    get_subscriber_info
)

from sros2.policy import _expression, _permission, _policy

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

    def main(self, *, args):
        policy = _policy.Policy(source=args.POLICY_FILE_PATH)
        node_names = []
        with NodeStrategy(args) as node:
            node_names = get_node_names(node=node, include_hidden_nodes=False)

        if not len(node_names):
            print('No nodes detected in the ROS graph. No policy file was generated.',
                  file=sys.stderr)
            return 1

        with DirectNode(args) as node:
            for node_name in node_names:
                profile = policy.get_or_create_profile(node_name.node, node_name.ns)

                subscribe_topics = get_subscriber_info(node=node, node_name=node_name)
                for topic in subscribe_topics:
                    _add_subscribe_topic_expression(profile, node_name, topic)

                publish_topics = get_publisher_info(node=node, node_name=node_name)
                for topic in publish_topics:
                    _add_publish_topic_expression(profile, node_name, topic)

                reply_services = get_service_info(node=node, node_name=node_name)
                for service in reply_services:
                    _add_reply_service_expression(profile, node_name, service)

                request_services = get_client_info(node=node, node_name=node_name)
                for service in request_services:
                    _add_request_service_expression(profile, node_name, service)

        with open(args.POLICY_FILE_PATH, 'w') as stream:
            policy.dump(stream)
        return 0


def _add_subscribe_topic_expression(profile, node_name, topic) -> None:
    profile.get_or_create_permission(
        _permission.PermissionType.TOPIC, _permission.PermissionRuleType.SUBSCRIBE,
        _permission.PermissionRuleQualifier.ALLOW
    ).add_expression(
        _expression.Expression.from_fields(
            node_name.fqn, node_name.ns, _expression.ExpressionType.TOPIC, topic.fqn
        )
    )


def _add_publish_topic_expression(profile, node_name, topic) -> None:
    profile.get_or_create_permission(
        _permission.PermissionType.TOPIC, _permission.PermissionRuleType.PUBLISH,
        _permission.PermissionRuleQualifier.ALLOW
    ).add_expression(
        _expression.Expression.from_fields(
            node_name.fqn, node_name.ns, _expression.ExpressionType.TOPIC, topic.fqn
        )
    )


def _add_reply_service_expression(profile, node_name, service) -> None:
    profile.get_or_create_permission(
        _permission.PermissionType.SERVICE, _permission.PermissionRuleType.REPLY,
        _permission.PermissionRuleQualifier.ALLOW
    ).add_expression(
        _expression.Expression.from_fields(
            node_name.fqn, node_name.ns, _expression.ExpressionType.SERVICE, service.fqn
        )
    )


def _add_request_service_expression(profile, node_name, service) -> None:
    profile.get_or_create_permission(
        _permission.PermissionType.SERVICE, _permission.PermissionRuleType.REQUEST,
        _permission.PermissionRuleQualifier.ALLOW
    ).add_expression(
        _expression.Expression.from_fields(
            node_name.fqn, node_name.ns, _expression.ExpressionType.SERVICE, service.fqn
        )
    )
