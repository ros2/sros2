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
import pathlib
import platform
import shutil

from cryptography import x509
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import ec

import sros2.errors

_DOMAIN_ID_ENV = 'ROS_DOMAIN_ID'
_KEYSTORE_DIR_ENV = 'ROS_SECURITY_KEYSTORE'


def create_symlink(*, src: pathlib.Path, dst: pathlib.Path) -> None:
    if dst.exists():
        # Don't do more work than we need to
        if dst.samefile(dst.parent.joinpath(src)):
            return
        dst.unlink()
    if platform.system() == 'Windows':
        # Resolve the absolute path for the source file
        actual_src = dst.parent.joinpath(src).resolve()
        try:
            # Create a hard link. To OpenSSL, this looks like a physical file.
            os.link(actual_src, dst)
        except OSError:
            # Failsafe: If the CI environment mounts enclaves on a different volume
            # than the public keystore, hard links will fail. Fallback to copy.
            shutil.copy2(actual_src, dst)
    else:
        # Retain standard, lightweight symlink behavior for Unix/macOS
        os.symlink(src, dst)


def domain_id() -> str:
    return os.getenv(_DOMAIN_ID_ENV, '0')


def get_keystore_path_from_env() -> pathlib.Path:
    root_keystore_path = os.getenv(_KEYSTORE_DIR_ENV)
    if root_keystore_path is None:
        raise sros2.errors.InvalidKeystoreEnvironmentError(_KEYSTORE_DIR_ENV)
    return pathlib.Path(root_keystore_path)


def create_smime_signed_file(
    cert_path: pathlib.Path,
    key_path: pathlib.Path,
    unsigned_file_path: pathlib.Path,
    signed_file_path: pathlib.Path,
) -> None:
    # Load the CA cert and key from disk
    cert = load_cert(cert_path)

    private_key = serialization.load_pem_private_key(key_path.read_bytes(), None)

    # Get the contents of the unsigned file, which we're about to sign
    content = unsigned_file_path.read_bytes()

    # Sign the contents, and write the result to the appropriate place
    signed_file_path.write_bytes(_sign_bytes(cert, private_key, content))


def build_key_and_cert(
    subject_name: x509.Name, *, ca: bool = False, ca_key=None, issuer_name=''
) -> tuple[x509.Certificate, ec.EllipticCurvePrivateKey]:
    if not issuer_name:
        issuer_name = subject_name

    # DDS-Security section 9.3.1 calls for prime256v1, for which SECP256R1 is an alias
    private_key = ec.generate_private_key(ec.SECP256R1())
    if not ca_key:
        ca_key = private_key

    if ca:
        extension = x509.BasicConstraints(ca=True, path_length=1)
    else:
        extension = x509.BasicConstraints(ca=False, path_length=None)

    utcnow = datetime.datetime.now(datetime.timezone.utc)
    builder = x509.CertificateBuilder(
        ).issuer_name(
            issuer_name
        ).serial_number(
            x509.random_serial_number()
        ).not_valid_before(
            utcnow
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
    cert = builder.sign(ca_key, hashes.SHA256())

    return (cert, private_key)


def write_key(
    key,
    key_path: pathlib.Path,
    *,
    encoding=serialization.Encoding.PEM,
    serialization_format=serialization.PrivateFormat.PKCS8,
    encryption_algorithm=serialization.NoEncryption()
) -> None:
    key_path.write_bytes(key.private_bytes(
        encoding=encoding,
        format=serialization_format,
        encryption_algorithm=encryption_algorithm))


def write_cert(cert, cert_path: pathlib.Path, *, encoding=serialization.Encoding.PEM) -> None:
    cert_path.write_bytes(cert.public_bytes(encoding=encoding))


def load_cert(cert_path: pathlib.Path) -> x509.Certificate:
    return x509.load_pem_x509_certificate(cert_path.read_bytes())


def _sign_bytes_pkcs7(cert, key, byte_string: bytes) -> bytes:
    from cryptography.hazmat.primitives.serialization import pkcs7

    builder = (
        pkcs7.PKCS7SignatureBuilder()
        .set_data(byte_string)
        .add_signer(cert, key, hashes.SHA256())
    )
    options = [pkcs7.PKCS7Options.Text, pkcs7.PKCS7Options.DetachedSignature]
    return builder.sign(serialization.Encoding.SMIME, options)


def _sign_bytes_ssl_binding(cert, key, byte_string: bytes) -> bytes:
    from cryptography.hazmat.bindings.openssl.binding import Binding as SSLBinding

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


def _sign_bytes(cert, key, byte_string: bytes) -> bytes:
    try:
        return _sign_bytes_pkcs7(cert, key, byte_string)
    except ImportError:
        pass

    return _sign_bytes_ssl_binding(cert, key, byte_string)
