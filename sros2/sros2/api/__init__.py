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

from collections import namedtuple

HIDDEN_NODE_PREFIX = '_'

NodeName = namedtuple('NodeName', ('node', 'ns', 'fqn'))
TopicInfo = namedtuple('Topic', ('fqn', 'type'))


def get_node_names(*, node, include_hidden_nodes=False):
    node_names_and_namespaces = node.get_node_names_and_namespaces()
    return [
        NodeName(
            node=t[0],
            ns=t[1],
            fqn=t[1] + ('' if t[1].endswith('/') else '/') + t[0])
        for t in node_names_and_namespaces
        if (
            include_hidden_nodes or
            (t[0] and not t[0].startswith(HIDDEN_NODE_PREFIX))
        )
    ]


def get_topics(node_name, func):
    names_and_types = func(node_name.node, node_name.ns)
    return [
        TopicInfo(
            fqn=t[0],
            type=t[1])
        for t in names_and_types]


def get_subscriber_info(node, node_name):
    return get_topics(node_name, node.get_subscriber_names_and_types_by_node)


def get_publisher_info(node, node_name):
    return get_topics(node_name, node.get_publisher_names_and_types_by_node)


def get_service_info(node, node_name):
    return get_topics(node_name, node.get_service_names_and_types_by_node)


def get_client_info(node, node_name):
    return get_topics(node_name, node.get_client_names_and_types_by_node)


def distribute_key(source_keystore_path, taget_keystore_path):
    raise NotImplementedError()
