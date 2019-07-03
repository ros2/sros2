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

import enum

from lxml import etree

from . import PolicyError


class UnsupportedExpressionTypeError(PolicyError):

    def __init__(self, type_string: str):
        super().__init__('Unsupported expression type: {!r}'.format(type_string))
        self.type = type_string


@enum.unique
class ExpressionType(enum.Enum):
    # Enum values should map to their XML values
    TOPIC = 'topic'
    SERVICE = 'service'


class Expression:
    """Class representation of an expression within a permission of an XML security policy."""

    @classmethod
    def from_fields(
            cls, fqn: str, namespace: str, expression_type: ExpressionType,
            text: str) -> 'Expression':
        """
        Create new Expression instance from its fields.

        :param str fqn: The node FQN.
        :param str namespace: The node namespace.
        :param ExpressionType expression_type: Type of expression.
        :param str text: Text of expression.

        :returns: The newly-created expression.
        :rtype: Expression
        """
        expression = etree.Element(expression_type.value)
        namespace = namespace.rstrip('/')

        if text.startswith(fqn + '/'):
            text = '~' + text[len(fqn + '/'):]
        elif text.startswith(namespace + '/'):
            text = text[len(namespace + '/'):]

        expression.text = text

        return cls(expression)

    def __init__(self, expression: etree.Element) -> None:
        """
        Create new Expression instance.

        :param etree.Element expression: XML Element containing the expression.
        """
        self._expression = expression

    def get_type(self) -> ExpressionType:
        """
        Return the type of the expression.

        :rtype: ExpressionType
        """
        try:
            return ExpressionType(self._expression.tag)
        except ValueError as e:
            raise UnsupportedExpressionTypeError(self._expression.tag) from e

    def get_text(self) -> str:
        """
        Return the text of the expression.

        :rtype: str
        """
        return self._expression.text
