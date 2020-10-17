# Copyright 2020 Canonical Ltd
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

import lxml.etree


class SROS2Error(Exception):
    """Base class for all SROS2 exceptions."""


class KeystoreError(SROS2Error):
    """Base class for keystore-related exceptions."""


class InvalidKeystoreError(KeystoreError):
    """Exception raised when the keystore is invalid."""

    def __init__(self, path: pathlib.Path):
        self.path = path
        super().__init__(f"'{str(path)}' is not a valid keystore")


class KeystoreExistsError(KeystoreError):
    """Exception raised when trying to create a keystore that already exists."""

    def __init__(self, path: pathlib.Path):
        self.path = path
        super().__init__(f"keystore '{str(path)}' already exists")


class EnclaveError(SROS2Error):
    """Base class for enclave-related exceptions."""


class InvalidEnclaveNameError(EnclaveError):
    """Exception raised when an enclave name is invalid."""

    def __init__(self, name: str):
        self.name = name
        super().__init__(f"'{name}' is not a valid enclave name")


class XmlError(SROS2Error):
    """Base class for XML-related exceptions."""


class InvalidGovernanceXMLError(XmlError):
    """Exception raised when governance XML is invalid."""

    def __init__(self, error: lxml.etree.DocumentInvalid):
        self.error = error
        super().__init__(f'invalid governance XML: {str(error)}')


class InvalidPermissionsXMLError(XmlError):
    """Exception raised when permissions XML is invalid."""

    def __init__(self, error: lxml.etree.DocumentInvalid):
        self.error = error
        super().__init__(f'invalid permissions XML: {str(error)}')


class SROS2EnvironmentError(SROS2Error):
    """Base class for environment-related exceptions."""


class InvalidKeystoreEnvironmentError(KeystoreError):
    """Exception raised when the keystore environment variable isn't valid."""

    def __init__(self, variable_name: str):
        self.variable_name = variable_name
        super().__init__(f"'{variable_name}' isn't pointing at a valid keystore")
