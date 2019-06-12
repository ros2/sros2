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

from rclpy.duration import Duration
from ros2cli.node.direct import DirectNode

from sros2.verb import VerbExtension

Event = namedtuple('Event', ('node_name', 'permission_type', 'rule_type', 'expression'))


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
    rule_qualifiers = set()
    for permission_group in permission_groups:
        expression_in_group = False
        for elem in permission_group:
            if getFQN(event.node_name, elem.text) == getFQN(event.node_name, event.expression):
                expression_in_group = True
                break
        if expression_in_group:
            rule_qualifiers.add(permission_group.attrib[event.rule_type])
    return EventPermission.reduce(rule_qualifiers)


class AmendPolicyVerb(VerbExtension):
    """Interactively add missing permissions to a permission file."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'POLICY_FILE_PATH', help='path of the policy xml file')
        arg.completer = FilesCompleter(
            allowednames=('xml'), directories=False)
        parser.add_argument(
            '--time-out', '-t',
            default=int(9999), type=int,
            help='a duration for monitoring the events (seconds)')

    def getEvents(self):
        pass

    def getPolicyEventStatus(self, policy, event):
        # Find all profiles for the node in the event
        profiles = policy.findall(
            path='profiles/profile[@ns="{ns}"][@node="{node}"]'.format(
                ns=event.node_name.ns,
                node=event.node_name.node))
        event_permissions = [getEventPermissionForProfile(p, event) for p in profiles]
        return EventPermission.reduce(event_permissions)

    def addPermission(self, event):
        pass

    def main(self, *, args):
        node = DirectNode(args)

        time_point_final = node.get_clock().now() + Duration(seconds=args.time_out)

        try:
            while (node._clock.now() < time_point_final):
                print(node._clock.now(), ' < ', time_point_final)
                # TODO(artivis) use rate once available
                time.sleep(0.25)
        except KeyboardInterrupt:
            print('done.')
