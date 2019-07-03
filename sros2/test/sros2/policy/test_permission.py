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

import pytest

from sros2.policy import _expression, _permission


def test_topic_publish_allow_from_xml():
    permission_xml = etree.fromstring(textwrap.dedent("""\
        <topics publish="ALLOW">
            <topic>test_topic</topic>
        </topics>
    """))

    permission = _permission.Permission(permission_xml)
    _assert_topic_publish(permission)
    _assert_topic_expressions(permission)


def test_topic_subscribe_allow_from_xml():
    permission_xml = etree.fromstring(textwrap.dedent("""\
        <topics subscribe="ALLOW">
            <topic>test_topic</topic>
        </topics>
    """))

    permission = _permission.Permission(permission_xml)
    _assert_topic_subscribe(permission)
    _assert_topic_expressions(permission)


def test_service_reply_allow_from_xml():
    permission_xml = etree.fromstring(textwrap.dedent("""\
        <services reply="ALLOW">
            <service>test_service</service>
        </services>
    """))

    permission = _permission.Permission(permission_xml)
    _assert_service_reply(permission)
    _assert_service_expressions(permission)


def test_topic_publish_allow_from_fields():
    _assert_topic_publish(_permission.Permission.from_fields(
        _permission.PermissionType.TOPIC, _permission.PermissionRuleType.PUBLISH,
        _permission.PermissionRuleQualifier.ALLOW))


def test_topic_subscribe_allow_from_fields():
    _assert_topic_subscribe(_permission.Permission.from_fields(
        _permission.PermissionType.TOPIC, _permission.PermissionRuleType.SUBSCRIBE,
        _permission.PermissionRuleQualifier.ALLOW))


def test_service_reply_allow_from_fields():
    _assert_service_reply(_permission.Permission.from_fields(
        _permission.PermissionType.SERVICE, _permission.PermissionRuleType.REPLY,
        _permission.PermissionRuleQualifier.ALLOW))


def test_unsupported_permission():
    permission_xml = etree.fromstring(textwrap.dedent("""\
        <topics publish="ALLOW" subscribe="ALLOW">
            <topic>test_topic</topic>
        </topics>
    """))

    permission = _permission.Permission(permission_xml)

    with pytest.raises(_permission.UnsupportedPolicyError):
        permission.get_rule_type()


def test_add_expression():
    permission_xml = etree.fromstring(textwrap.dedent("""\
        <topics publish="ALLOW">
        </topics>
    """))

    permission = _permission.Permission(permission_xml)
    assert len(permission.get_expressions()) == 0

    permission.add_expression(_expression.Expression.from_fields(
        '/test_node', '/', _expression.ExpressionType.TOPIC, 'test_topic'))

    _assert_topic_publish(permission)


def _assert_topic_publish(permission):
    assert permission.get_type() == _permission.PermissionType.TOPIC
    assert permission.get_rule_type() == _permission.PermissionRuleType.PUBLISH
    assert permission.get_rule_qualifier() == _permission.PermissionRuleQualifier.ALLOW


def _assert_topic_subscribe(permission):
    assert permission.get_type() == _permission.PermissionType.TOPIC
    assert permission.get_rule_type() == _permission.PermissionRuleType.SUBSCRIBE
    assert permission.get_rule_qualifier() == _permission.PermissionRuleQualifier.ALLOW


def _assert_topic_expressions(permission):
    expressions = permission.get_expressions()
    assert len(expressions) == 1

    expression = expressions[0]
    assert expression.get_type() == _expression.ExpressionType.TOPIC
    assert expression.get_text() == 'test_topic'


def _assert_service_reply(permission):
    assert permission.get_type() == _permission.PermissionType.SERVICE
    assert permission.get_rule_type() == _permission.PermissionRuleType.REPLY
    assert permission.get_rule_qualifier() == _permission.PermissionRuleQualifier.ALLOW


def _assert_service_expressions(permission):
    expressions = permission.get_expressions()
    assert len(expressions) == 1

    expression = expressions[0]
    assert expression.get_type() == _expression.ExpressionType.SERVICE
    assert expression.get_text() == 'test_service'
