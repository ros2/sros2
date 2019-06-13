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
import os
import time

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

from rclpy.duration import Duration
from ros2cli.node.direct import DirectNode

from sros2.api import (
    get_publisher_info,
    get_service_info,
    get_subscriber_info,
    NodeName
)

from sros2.policy import (
    dump_policy,
    load_policy,
    POLICY_VERSION
)

from sros2.verb import VerbExtension

POLICY_FILE_NOT_FOUND = 'Package policy file not found'
POLICY_FILE_NOT_VALID = 'Package policy file not valid'

Event = namedtuple('Event',
                   ('node_name', 'permission_type', 'rule_type', 'expression'))


def getFQN(node_name, expression):
    """Return an expression's fully qualified name."""
    fqn = expression
    # Expression name is already fully qualified
    if expression.startswith('/'):
        fqn = expression
    # Fully qualified with uri
    elif expression.startswith('rostopic://'):
        fqn = '/' + expression[len('rostopic://'):]
    # Private name
    elif expression.startswith('~'):
        fqn = node_name.fqn + '/' + expression[len('~'):]
    # Relative or base name
    else:
        fqn = node_name.ns + '/' + expression
    return fqn


class EventList:
    subscribe = []
    publish = []
    reply_service = []


class EventPermission:
    ALLOW = 'ALLOW'
    DENY = 'DENY'
    NOT_SPECIFIED = 'NOT_SPECIFIED'

    @staticmethod
    def reduce(rule_qualifiers):
        if EventPermission.DENY in rule_qualifiers:
            return EventPermission.DENY
        if EventPermission.ALLOW in rule_qualifiers:
            return EventPermission.ALLOW
        else:
            return EventPermission.NOT_SPECIFIED


def getEventPermissionForProfile(profile, event):
    permission_groups = profile.findall(
            path='{permission_type}s[@{rule_type}]'.format(
                permission_type=event.permission_type,
                rule_type=event.rule_type))
    # Base case is that permissions for the event is not specified
    rule_qualifiers = set(EventPermission.NOT_SPECIFIED)
    for permission_group in permission_groups:
        expression_in_group = False
        for elem in permission_group:
            if getFQN(event.node_name, elem.text) == getFQN(event.node_name,
                                                            event.expression):
                expression_in_group = True
                break
        if expression_in_group:
            rule_qualifiers.add(permission_group.attrib[event.rule_type])
    return EventPermission.reduce(rule_qualifiers)


class AmendPolicyVerb(VerbExtension):
    """Interactively add missing permissions to a permission file."""

    def __init__(self):
        self.event_cache = []
        self.profile = None

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'policy_file_path', help='path of the policy xml file')
        arg.completer = FilesCompleter(
            allowednames=('xml'), directories=False)
        parser.add_argument(
            '--time-out', '-t',
            default=int(9999), type=int,
            help='a duration for monitoring the events (seconds)')

    def get_policy(policy_file_path):
        """Return a policy tree from the path or an empty policy tree if it doens't exist."""
        if os.path.isfile(policy_file_path):
            return load_policy(policy_file_path)
        else:
            profiles = etree.Element('profiles')
            policy = etree.Element('policy')
            policy.attrib['version'] = POLICY_VERSION
            policy.append(profiles)
            return policy

    @staticmethod
    def get_profiles(policy, node_name):
        """Return all profiles for the given node in policy."""
        profiles = policy.findall(
            path='profiles/profile[@ns="{ns}"][@node="{node}"]'.format(
                ns=node_name.ns,
                node=node_name.node))
        if not profiles:
            profile = etree.Element('profile')
            profile.attrib['ns'] = node_name.ns
            profile.attrib['node'] = node_name.node
            all_profiles = policy.find('profiles')
            if not all_profiles:
                all_profiles = etree.Element('profiles')
                policy.append(all_profiles)
            all_profiles.append(profile)
            profiles = [profile]
        return profiles

    @staticmethod
    def get_permission_groups(profile, permission_type, rule_type, rule_qualifier):
        """Return all permission groups in a profile with the specified properties."""
        return profile.findall(
            path='{permission_type}s[@{rule_type}="{rule_qualifier}"]'.format(
                permission_type=permission_type,
                rule_type=rule_type,
                rule_qualifier=rule_qualifier))

    @staticmethod
    def create_permission(event):
        expression_fqn = getFQN(event.node_name, event.expression)
        permission = etree.Element(event.permission_type)
        if expression_fqn.startswith(event.node_name.fqn + '/'):
            permission.text = '~' + expression_fqn[len(event.node_name.fqn + '/'):]
        elif expression_fqn.startswith(event.node_name.ns + '/'):
            permission.text = expression_fqn[len(event.node_name.ns + '/'):]
        elif expression_fqn.count('/') == 1 and event.node_name.ns == '/':
            permission.text = expression_fqn[len('/'):]
        else:
            permission.text = expression_fqn
        return permission

    @staticmethod
    def add_permission(policy, event, rule_qualifier):
        """Add a permission to the policy for the given event."""
        profiles = AmendPolicyVerb.get_profiles(policy, event.node_name)
        profiles_with_permissions = [p for p in profiles if AmendPolicyVerb.get_permission_groups(
            p, event.permission_type, event.rule_type, rule_qualifier)]
        profile = profiles[0] if not profiles_with_permissions else profiles_with_permissions[0]

        permission_groups = AmendPolicyVerb.get_permission_groups(
            profile, event.permission_type, event.rule_type, rule_qualifier)
        if len(permission_groups) == 0:
            permission_group = etree.Element(event.permission_type + 's')
            permission_group.attrib[event.rule_type] = rule_qualifier
            profile.append(permission_group)
        else:
            permission_group = permission_groups[0]

        permission_group.append(AmendPolicyVerb.create_permission(event))

    def getEvents(self, node, node_name):
        events = EventList
        subscribe_topics = get_subscriber_info(node=node, node_name=node_name)
        if subscribe_topics:
            events.subscribe = events.subscribe + subscribe_topics
        publish_topics = get_publisher_info(node=node, node_name=node_name)
        if publish_topics:
            events.publish = events.publish + publish_topics
        reply_services = get_service_info(node=node, node_name=node_name)
        if reply_services:
            events.reply_service = events.reply_service + reply_services

    def getPolicyEventStatus(self, policy, event):
        # Find all profiles for the node in the event
        profiles = policy.findall(
            path='profiles/profile[@ns="{ns}"][@node="{node}"]'.format(
                ns=event.node_name.ns,
                node=event.node_name.node))

        event_permissions = \
            [getEventPermissionForProfile(p, event) for p in profiles]

        return EventPermission.reduce(event_permissions)

    def filterEvents(self, events):

        if events is None:
            return []

        not_cached_events = list(set(events).difference(
                                self.event_cache))

        filtered_events = []
        for not_cached_event in not_cached_events:
            print(not_cached_event)
            print(getEventPermissionForProfile(self.profile, not_cached_event))
            if (
                getEventPermissionForProfile(self.profile, not_cached_event) ==
                EventPermission.ALLOW
            ):
                self.keepCached(not_cached_event)
            else:
                filtered_events.append(not_cached_event)

        return filtered_events

    def keepCached(self, event):
        self.event_cache.append(event)

    def promptUserAboutPermission(self, event):
        usr_input = None
        while usr_input not in ['Y', 'y', 'N', 'n', '']:
            print('Event: ', event)
            usr_input = input('Do you want to add this event '
                              'to the permission list? (Y/n) : ')

        if usr_input in ['Y', 'y', '']:
            self.add_permission(event)
            print('Permission granted !')
        elif usr_input in ['N', 'n']:
            print('Permission denied !')

        self.keepCached(event)

        print('\n')

    def main(self, *, args):
        try:
            self.profile = load_policy(args.policy_file_path)
        except FileNotFoundError:
            return POLICY_FILE_NOT_FOUND
        except RuntimeError:
            return POLICY_FILE_NOT_VALID

        node = DirectNode(args)

        time_point_final = node.get_clock().now() + \
            Duration(seconds=args.time_out)

        try:
            while (node._clock.now() < time_point_final):
                print('Scanning for events...')  # , end='\r'

                # TODO(artivis) get_node_name
                node_name = NodeName('node', '/ns', '/ns/node')
                events = self.getEvents(node, node_name)

                filtered_events = self.filterEvents(events)

                for filtered_event in filtered_events:
                    self.promptUserAboutPermission(filtered_event)

                # TODO(artivis) use rate once available
                time.sleep(0.25)

            with open(args.policy_file_path, 'w') as stream:
                dump_policy(self.profile, stream)
        except KeyboardInterrupt:
            pass
