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

import io
import textwrap

import pytest

from sros2.policy import _expression, _permission, _policy, _profile, InvalidPolicyError


def test_policy():
    contents = textwrap.dedent("""\
        <policy version="0.1.0">
            <profiles>
                <profile ns="/ns" node="test_node">
                    <topics publish="ALLOW">
                        <topic>pub_topic</topic>
                    </topics>
                </profile>
            </profiles>
        </policy>
    """)

    with io.StringIO(contents) as s:
        policy = _policy.Policy(source=s)

    assert policy.get_version() == '0.1.0'

    assert len(policy.get_profiles()) == 1
    profile = policy.get_profile('test_node', '/ns')
    assert profile is not None


def test_get_profile_not_found():
    contents = textwrap.dedent("""\
        <policy version="0.1.0">
            <profiles>
                <profile ns="/ns" node="test_node">
                    <topics publish="ALLOW">
                        <topic>pub_topic</topic>
                    </topics>
                </profile>
            </profiles>
        </policy>
    """)

    with io.StringIO(contents) as s:
        policy = _policy.Policy(source=s)

    profile = policy.get_profile('another_node', '/ns')
    assert profile is None


def test_dump_invalid_policy():
    policy = _policy.Policy()
    with io.StringIO() as s:
        with pytest.raises(InvalidPolicyError):
            policy.dump(s)


def test_add_profile():
    policy = _policy.Policy()
    assert len(policy.get_profiles()) == 0

    profile = _profile.Profile.from_fields('test_node', '/ns')
    policy.add_profile(profile)
    assert len(profile.get_permissions()) == 0

    permission = _permission.Permission.from_fields(
        _permission.PermissionType.TOPIC, _permission.PermissionRuleType.PUBLISH,
        _permission.PermissionRuleQualifier.ALLOW)
    assert len(permission.get_expressions()) == 0

    expression = _expression.Expression.from_fields(
        '/test_node', '/ns', _expression.ExpressionType.TOPIC, 'test_topic')

    permission.add_expression(expression)
    profile.add_permission(permission)

    # Verify that the above modification is reflected in the profile
    profile = policy.get_profile('test_node', '/ns')
    assert profile is not None

    permission = profile.get_permission(
        _permission.PermissionType.TOPIC, _permission.PermissionRuleType.PUBLISH,
        _permission.PermissionRuleQualifier.ALLOW)
    assert permission is not None

    expressions = permission.get_expressions()
    assert len(expressions) == 1

    expression = expressions[0]
    assert expression.get_type() == _expression.ExpressionType.TOPIC
    assert expression.get_text() == 'test_topic'
