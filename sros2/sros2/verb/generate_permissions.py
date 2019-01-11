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

from collections import defaultdict

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import NodeStrategy
from sros2.api import get_node_names
from sros2.api import get_publisher_info
from sros2.api import get_service_info
from sros2.api import get_subscriber_info
from sros2.verb import VerbExtension


def formatTopics(topic_list, permission, topic_map):
    for topic in topic_list:
        topic_map[topic.name].append(permission)


class GeneratePermissionsVerb(VerbExtension):
    """Generate permissions."""

    def add_arguments(self, parser, cli_name):

        arg = parser.add_argument(
            'POLICY_FILE_PATH', help='path of the permission yaml file')
        arg.completer = FilesCompleter(
            allowednames=('yaml'), directories=False)

    def main(self, *, args):
        node_names = []
        with NodeStrategy(args) as node:
            node_names = get_node_names(node=node, include_hidden_nodes=False)
        policy_dict = {}
        with DirectNode(args) as node:
            for node_name in node_names:
                subscribers = get_subscriber_info(node=node, node_name=node_name)
                publishers = get_publisher_info(node=node, node_name=node_name)
                services = get_service_info(node=node, node_name=node_name)
                topic_map = defaultdict(list)
                formatTopics(publishers, 'publish', topic_map)
                formatTopics(subscribers, 'subscribe', topic_map)
                formatted_topic_map = {}
                for topic_name, permission_list in topic_map.items():
                    formatted_topic_map[topic_name] = {'allow': permission_list}
                service_map = defaultdict(list)
                formatTopics(services, 'reply', service_map)
                formatted_services_map = {}
                for service, permission_list in service_map.items():
                    formatted_services_map[service] = {'allow': permission_list}
                policy_dict[node_name.name] = {'topics': formatted_topic_map}
                policy_dict[node_name.name]['services'] = formatted_services_map
        import yaml
        from io import open
        formatted_policy_dict = {'nodes': policy_dict}
        if policy_dict:
            with open(args.POLICY_FILE_PATH, 'w') as stream:
                yaml.dump(formatted_policy_dict, stream, default_flow_style=False)
        else:
            print('No nodes found to generate policies')
