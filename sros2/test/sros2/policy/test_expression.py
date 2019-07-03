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

from sros2.policy import _expression


def test_topic_from_xml():
    expression_xml = etree.fromstring(textwrap.dedent("""\
        <topic>test_topic</topic>
    """))

    _assert_topic(_expression.Expression(expression_xml))


def test_service_from_xml():
    expression_xml = etree.fromstring(textwrap.dedent("""\
        <service>test_service</service>
    """))

    _assert_service(_expression.Expression(expression_xml))


def test_unsupported_type_from_xml():
    expression_xml = etree.fromstring(textwrap.dedent("""\
        <unsupported>test</unsupported>
    """))

    expression = _expression.Expression(expression_xml)

    with pytest.raises(_expression.UnsupportedExpressionTypeError) as e:
        expression.get_type()
    assert e.value.type == 'unsupported'


def test_topic_from_fields():
    _assert_topic(_expression.Expression.from_fields(
        '/test_node', '/', _expression.ExpressionType.TOPIC, 'test_topic'))


def test_service_from_fields():
    _assert_service(_expression.Expression.from_fields(
        '/test_node', '/', _expression.ExpressionType.SERVICE, 'test_service'))


def test_nested_under_node():
    expression = _expression.Expression.from_fields(
        '/test_node', '/', _expression.ExpressionType.TOPIC, '/test_node/test_topic')
    assert expression.get_type() == _expression.ExpressionType.TOPIC
    assert expression.get_text() == '~test_topic'

    expression = _expression.Expression.from_fields(
        '/test_node', '/', _expression.ExpressionType.TOPIC, '/test_node/level2/test_topic')
    assert expression.get_type() == _expression.ExpressionType.TOPIC
    assert expression.get_text() == '~level2/test_topic'


def test_nested_under_namespace():
    expression = _expression.Expression.from_fields(
        '/test_node', '/ns', _expression.ExpressionType.TOPIC, '/ns/test_topic')
    assert expression.get_type() == _expression.ExpressionType.TOPIC
    assert expression.get_text() == 'test_topic'

    # The special case of namespace being '/' should be properly handled: topics within the
    # namespace should still be relative in the profile.
    expression = _expression.Expression.from_fields(
        '/test_node', '/', _expression.ExpressionType.TOPIC, '/another_ns/test_topic')
    assert expression.get_type() == _expression.ExpressionType.TOPIC
    assert expression.get_text() == 'another_ns/test_topic'

    expression = _expression.Expression.from_fields(
        '/test_node', '/', _expression.ExpressionType.TOPIC, '/test_topic')
    assert expression.get_type() == _expression.ExpressionType.TOPIC
    assert expression.get_text() == 'test_topic'


def _assert_topic(expression):
    assert expression.get_type() == _expression.ExpressionType.TOPIC
    assert expression.get_text() == 'test_topic'


def _assert_service(expression):
    assert expression.get_type() == _expression.ExpressionType.SERVICE
    assert expression.get_text() == 'test_service'
