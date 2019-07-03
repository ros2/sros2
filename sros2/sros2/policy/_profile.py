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

from typing import List, Optional

from lxml import etree

from ._permission import Permission, PermissionRuleQualifier, PermissionRuleType, PermissionType


class Profile:
    """Class representation of a profile within an XML security policy."""

    @classmethod
    def from_fields(cls, node_name: str, namespace: str) -> 'Profile':
        """
        Create new Profile instance from its fields.

        :param str node_name: The name of the profile's node
        :param str namespace: The namespace of the profile's node.

        :returns: The newly-created profile.
        :rtype: Profile
        """
        profile = etree.Element('profile')
        profile.attrib['ns'] = namespace
        profile.attrib['node'] = node_name

        return cls(profile)

    def __init__(self, profile: etree.Element) -> None:
        """
        Create new Profile instance.

        :param etree.Element profile: XML Element containing the profile.
        """
        self._profile = profile

    def get_node(self) -> str:
        """
        Return the name of the profile's node.

        :rtype: str
        """
        return self._profile.get('node')

    def get_namespace(self) -> str:
        """
        Return the namespace of the profile's node.

        :rtype: str
        """
        return self._profile.get('ns')

    def get_permissions(self) -> List[Permission]:
        """
        Return all permissions making up the profile.

        :rtype: list
        """
        permissions = []
        for child in self._profile:
            permissions.append(Permission(child))

        return permissions

    def get_permission(
            self, permission_type: PermissionType, rule_type: PermissionRuleType,
            rule_qualifier: PermissionRuleQualifier) -> Optional[Permission]:
        """
        Get permission that matches provided parameters.

        :param PermissionType permission_type: The type of permission.
        :param PermissionRuleType rule_type: The type of rule.
        :param PermissionRuleQualifier rule_qualifier: Qualifier for the rule.

        If no matching permission exists, this function will return None.
        """
        permission_xml = self._profile.find(
            path='{permission_type}[@{rule_type}="{rule_qualifier}"]'.format(
                permission_type=permission_type.value,
                rule_type=rule_type.value,
                rule_qualifier=rule_qualifier.value))

        if permission_xml is not None:
            return Permission(permission_xml)
        else:
            return None

    def add_permission(self, permission: Permission) -> Permission:
        """
        Add permission to the profile.

        :param Permission permission: Permission to be added.

        :returns: The permission that was added
        :rtype: Permission

        Future modifications of the permission will be reflected in this profile once this
        function is called.
        """
        self._profile.append(permission._permission)
        return permission

    def get_or_create_permission(
            self, permission_type: PermissionType, rule_type: PermissionRuleType,
            rule_qualifier: PermissionRuleQualifier) -> Permission:
        """
        Get matching permission or create new one if it doesn't exist.

        :param PermissionType permission_type: The type of permission.
        :param PermissionRuleType rule_type: The type of rule.
        :param PermissionRuleQualifier rule_qualifier: Qualifier for the rule.

        Future modifications of the permission will be reflected in this profile once this
        function is called.
        """
        permission = self.get_permission(permission_type, rule_type, rule_qualifier)
        if not permission:
            permission = self.add_permission(
                Permission.from_fields(permission_type, rule_type, rule_qualifier))
        return permission
