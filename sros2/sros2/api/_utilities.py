# Copyright 2020 Canonical Ltd
# Copyright 2016-2019 Open Source Robotics Foundation, Inc.
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

import datetime
import os
import sys

from cryptography import x509
from cryptography.hazmat.backends import default_backend as cryptography_backend
from cryptography.hazmat.bindings.openssl.binding import Binding as SSLBinding
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import ec

_DOMAIN_ID_ENV = 'ROS_DOMAIN_ID'
_KEYSTORE_DIR_ENV = 'ROS_SECURITY_KEYSTORE'


def create_symlink(*, src, dst):
    if os.path.exists(dst):
        src_abs_path = os.path.join(os.path.dirname(dst), src)
        if os.path.samefile(src_abs_path, dst):
            return
        print(f"Existing symlink '{dst}' does not match '{src_abs_path}', overriding it!")
        os.remove(dst)
    os.symlink(src=src, dst=dst)


def domain_id() -> str:
    return os.getenv(_DOMAIN_ID_ENV, '0')


def get_keystore_path_from_env():
    root_keystore_path = os.getenv(_KEYSTORE_DIR_ENV)
    if root_keystore_path is None:
        print('%s is empty' % _KEYSTORE_DIR_ENV, file=sys.stderr)
    return root_keystore_path


def create_smime_signed_file(cert_path, key_path, unsigned_file_path, signed_file_path):
    # Load the CA cert and key from disk
    cert = load_cert(cert_path)

    with open(key_path, 'rb') as key_file:
        private_key = serialization.load_pem_private_key(
            key_file.read(), None, cryptography_backend())

    # Get the contents of the unsigned file, which we're about to sign
    with open(unsigned_file_path, 'rb') as f:
        content = f.read()

    # Sign the contents, and write the result to the appropriate place
    with open(signed_file_path, 'wb') as f:
        f.write(_sign_bytes(cert, private_key, content))


def build_key_and_cert(subject_name, *, ca=False, ca_key=None, issuer_name=''):
    if not issuer_name:
        issuer_name = subject_name

    # DDS-Security section 9.3.1 calls for prime256v1, for which SECP256R1 is an alias
    private_key = ec.generate_private_key(ec.SECP256R1, cryptography_backend())
    if not ca_key:
        ca_key = private_key

    if ca:
        extension = x509.BasicConstraints(ca=True, path_length=1)
    else:
        extension = x509.BasicConstraints(ca=False, path_length=None)

    utcnow = datetime.datetime.utcnow()
    builder = x509.CertificateBuilder(
        ).issuer_name(
            issuer_name
        ).serial_number(
            x509.random_serial_number()
        ).not_valid_before(
            # Using a day earlier here to prevent Connext (5.3.1) from complaining
            # when extracting it from the permissions file and thinking it's in the future
            # https://github.com/ros2/ci/pull/436#issuecomment-624874296
            utcnow - datetime.timedelta(days=1)
        ).not_valid_after(
            # TODO: This should not be hard-coded
            utcnow + datetime.timedelta(days=3650)
        ).public_key(
            private_key.public_key()
        ).subject_name(
            subject_name
        ).add_extension(
            extension, critical=ca
        )
    cert = builder.sign(ca_key, hashes.SHA256(), cryptography_backend())

    return (cert, private_key)


def write_key(
    key,
    key_path,
    *,
    encoding=serialization.Encoding.PEM,
    serialization_format=serialization.PrivateFormat.PKCS8,
    encryption_algorithm=serialization.NoEncryption()
):
    with open(key_path, 'wb') as f:
        f.write(key.private_bytes(
            encoding=encoding,
            format=serialization_format,
            encryption_algorithm=encryption_algorithm))


def write_cert(cert, cert_path, *, encoding=serialization.Encoding.PEM):
    with open(cert_path, 'wb') as f:
        f.write(cert.public_bytes(encoding=encoding))


def load_cert(cert_path):
    with open(cert_path, 'rb') as cert_file:
        return x509.load_pem_x509_certificate(
            cert_file.read(), cryptography_backend())


def _sign_bytes(cert, key, byte_string):
    # Using two flags here to get the output required:
    #   - PKCS7_DETACHED: Use cleartext signing
    #   - PKCS7_TEXT: Set the MIME headers for text/plain
    flags = SSLBinding.lib.PKCS7_DETACHED
    flags |= SSLBinding.lib.PKCS7_TEXT

    # Convert the byte string into a buffer for SSL
    bio_in = SSLBinding.lib.BIO_new_mem_buf(byte_string, len(byte_string))
    try:
        pkcs7 = SSLBinding.lib.PKCS7_sign(
            cert._x509, key._evp_pkey, SSLBinding.ffi.NULL, bio_in, flags)
    finally:
        # Free the memory allocated for the buffer
        SSLBinding.lib.BIO_free(bio_in)

    # PKCS7_sign consumes the buffer; allocate a new one again to get it into the final document
    bio_in = SSLBinding.lib.BIO_new_mem_buf(byte_string, len(byte_string))
    try:
        # Allocate a buffer for the output document
        bio_out = SSLBinding.lib.BIO_new(SSLBinding.lib.BIO_s_mem())
        try:
            # Write the final document out to the buffer
            SSLBinding.lib.SMIME_write_PKCS7(bio_out, pkcs7, bio_in, flags)

            # Copy the output document back to python-managed memory
            result_buffer = SSLBinding.ffi.new('char**')
            buffer_length = SSLBinding.lib.BIO_get_mem_data(bio_out, result_buffer)
            output = SSLBinding.ffi.buffer(result_buffer[0], buffer_length)[:]
        finally:
            # Free the memory required for the output buffer
            SSLBinding.lib.BIO_free(bio_out)
    finally:
        # Free the memory allocated for the input buffer
        SSLBinding.lib.BIO_free(bio_in)

    return output
