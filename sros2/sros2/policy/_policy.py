# Copyright 2019 Canonical Ltd
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
from typing import List, Optional, TextIO, Union

from lxml import etree

from . import dump_policy, load_policy, POLICY_VERSION
from ._profile import Profile


class Policy:
    """Class representation of an XML security policy."""

    def __init__(self, *, source: Union[TextIO, str] = None) -> None:
        """
        Create new Policy instance, optionally by loading an existing policy.

        :param source: Either the path to existing policy file to be loaded, or file-like
                       object containing existing policy.
        """
        # If source was provided, initialize this class with it. Otherwise, start a fresh
        # (empty) one.
        if source and (not isinstance(source, str) or os.path.isfile(source)):
            self._policy = load_policy(source).getroot()
        else:
            profiles = etree.Element('profiles')
            self._policy = etree.Element('policy')
            self._policy.attrib['version'] = POLICY_VERSION
            self._policy.append(profiles)

    def get_version(self) -> str:
        """
        Return the version of the policy.

        :rtype: str
        """
        return self._policy.get('version')

    def get_profiles(self) -> List[Profile]:
        """
        Return all profiles making up the policy.

        :rtype: list
        """
        profiles = []
        for child in self._policy.find('profiles'):
            profiles.append(Profile(child))

        return profiles

    def get_profile(self, node_name: str, namespace: str) -> Optional[Profile]:
        """
        Get profile that matches provided parameters.

        :param str node_name: The name of the profile's node
        :param str namespace: The namespace of the profile's node.

        If no matching profile exists, this function will return None.
        """
        xml_profile = self._policy.find(
            path='profiles/profile[@ns="{ns}"][@node="{node}"]'.format(
                ns=namespace,
                node=node_name))

        if xml_profile is not None:
            return Profile(xml_profile)
        else:
            return None

    def add_profile(self, profile: Profile) -> Profile:
        self._policy.find('profiles').append(profile._profile)
        return profile

    def get_or_create_profile(self, node_name: str, namespace: str) -> Profile:
        """
        Get matching profile or create new one if it doesn't exist.

        :param str node_name: The name of the profile's node
        :param str namespace: The namespace of the profile's node.

        Future modifications of the profile will be reflected in this policy once this
        function is called.
        """
        profile = self.get_profile(node_name, namespace)
        if not profile:
            profile = self.add_profile(Profile.from_fields(node_name, namespace))
        return profile

    def dump(self, stream: TextIO):
        """
        Dump policy to stream.

        :param TextIO stream: File like object to which policy will be written.
        """
        dump_policy(self._policy, stream)
