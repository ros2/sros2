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

import textwrap

from lxml import etree

from sros2.policy import _expression, _permission, _profile


def test_get_permission():
    profile_xml = etree.fromstring(textwrap.dedent("""\
        <profile ns="/" node="test_node">
            <topics publish="ALLOW">
                <topic>test_topic</topic>
            </topics>
        </profile>
    """))

    profile = _profile.Profile(profile_xml)
    assert profile.get_node() == 'test_node'
    assert profile.get_namespace() == '/'

    assert len(profile.get_permissions()) == 1
    permission = profile.get_permission(
        _permission.PermissionType.TOPIC, _permission.PermissionRuleType.PUBLISH,
        _permission.PermissionRuleQualifier.ALLOW)
    assert permission is not None


def test_from_fields():
    profile = _profile.Profile.from_fields('test_node', '/')
    assert profile.get_node() == 'test_node'
    assert profile.get_namespace() == '/'
    assert len(profile.get_permissions()) == 0


def test_get_permission_not_found():
    profile_xml = etree.fromstring(textwrap.dedent("""\
        <profile ns="/" node="test_node">
            <topics publish="ALLOW">
                <topic>test_topic</topic>
            </topics>
        </profile>
    """))

    profile = _profile.Profile(profile_xml)
    permission = profile.get_permission(
        _permission.PermissionType.TOPIC, _permission.PermissionRuleType.SUBSCRIBE,
        _permission.PermissionRuleQualifier.ALLOW)
    assert permission is None


def test_add_permission():
    profile_xml = etree.fromstring(textwrap.dedent("""\
        <profile ns="/" node="test_node">
        </profile>
    """))

    profile = _profile.Profile(profile_xml)
    assert len(profile.get_permissions()) == 0

    permission = _permission.Permission.from_fields(
        _permission.PermissionType.TOPIC, _permission.PermissionRuleType.PUBLISH,
        _permission.PermissionRuleQualifier.ALLOW)
    profile.add_permission(permission)

    assert len(permission.get_expressions()) == 0
    permission.add_expression(_expression.Expression.from_fields(
        '/test_node', '/', _expression.ExpressionType.TOPIC, 'test_topic'))

    # Verify that the above modification is reflected in the profile
    permission = profile.get_permission(
        _permission.PermissionType.TOPIC, _permission.PermissionRuleType.PUBLISH,
        _permission.PermissionRuleQualifier.ALLOW)
    assert permission is not None

    expressions = permission.get_expressions()
    assert len(expressions) == 1

    expression = expressions[0]
    assert expression.get_type() == _expression.ExpressionType.TOPIC
    assert expression.get_text() == 'test_topic'
