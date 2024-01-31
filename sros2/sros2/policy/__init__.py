# Copyright 2018-2019 Open Source Robotics Foundation, Inc.
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

import pathlib

from lxml import etree

try:
    import importlib.resources as importlib_resources
except ModuleNotFoundError:
    # (clalancette): This fallback is necessary when importlib-resources is installed from pip.
    # Note that we have to ignore the type, otherwise mypy throws a warning.
    # See https://github.com/python/mypy/issues/1153 for details.
    import importlib_resources  # type: ignore

POLICY_VERSION = '0.2.0'


def _get_path(template, name):
    if hasattr(importlib_resources, 'files'):
        return importlib_resources.files(template).joinpath(name)
    else:
        with importlib_resources.path(template, name) as path:
            return path


def get_policy_default(name: str) -> pathlib.Path:
    return _get_path('sros2.policy.defaults', name)


def get_policy_schema(name: str) -> pathlib.Path:
    return _get_path('sros2.policy.schemas', name)


def get_policy_template(name: str) -> pathlib.Path:
    return _get_path('sros2.policy.templates', name)


def get_transport_default(transport: str, name: str) -> pathlib.Path:
    module = 'sros2.policy.defaults.' + transport
    return _get_path(module, name)


def get_transport_schema(transport: str, name: str) -> pathlib.Path:
    module = 'sros2.policy.schemas.' + transport
    return _get_path(module, name)


def get_transport_template(transport: str, name: str) -> pathlib.Path:
    module = 'sros2.policy.templates.' + transport
    return _get_path(module, name)


def load_policy(policy_file_path: pathlib.Path) -> etree.ElementTree:
    if not policy_file_path.is_file():
        raise FileNotFoundError("policy file '%s' does not exist" % policy_file_path)
    policy = etree.parse(str(policy_file_path))
    policy.xinclude()
    try:
        policy_xsd_path = get_policy_schema('policy.xsd')
        policy_xsd = etree.XMLSchema(etree.parse(str(policy_xsd_path)))
        policy_xsd.assertValid(policy)
    except etree.DocumentInvalid as e:
        raise RuntimeError(str(e))
    return policy


def dump_policy(policy, stream) -> None:
    policy_xsl_path = get_policy_template('policy.xsl')
    policy_xsl = etree.XSLT(etree.parse(str(policy_xsl_path)))
    policy = policy_xsl(policy)
    try:
        policy_xsd_path = get_policy_schema('policy.xsd')
        policy_xsd = etree.XMLSchema(etree.parse(str(policy_xsd_path)))
        policy_xsd.assertValid(policy)
    except etree.DocumentInvalid as e:
        raise RuntimeError(str(e))
    stream.write(etree.tostring(policy, pretty_print=True).decode())
