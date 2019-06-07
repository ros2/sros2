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

from collections import namedtuple
import itertools
import os
import platform
import shutil
import subprocess
import sys

from lxml import etree

from rclpy.exceptions import InvalidNamespaceException
from rclpy.exceptions import InvalidNodeNameException
from rclpy.validate_namespace import validate_namespace
from rclpy.validate_node_name import validate_node_name

from sros2.policy import (
    get_policy_default,
    get_transport_default,
    get_transport_schema,
    get_transport_template,
    load_policy,
)

HIDDEN_NODE_PREFIX = '_'
DOMAIN_ID_ENV = 'ROS_DOMAIN_ID'

NodeName = namedtuple('NodeName', ('node', 'ns', 'fqn'))
TopicInfo = namedtuple('Topic', ('fqn', 'type'))


def get_node_names(*, node, include_hidden_nodes=False):
    node_names_and_namespaces = node.get_node_names_and_namespaces()
    return [
        NodeName(
            node=t[0],
            ns=t[1],
            fqn=t[1] + ('' if t[1].endswith('/') else '/') + t[0])
        for t in node_names_and_namespaces
        if (
            include_hidden_nodes or
            (t[0] and not t[0].startswith(HIDDEN_NODE_PREFIX))
        )
    ]


def get_topics(node_name, func):
    names_and_types = func(node_name.node, node_name.ns)
    return [
        TopicInfo(
            fqn=t[0],
            type=t[1])
        for t in names_and_types]


def get_subscriber_info(node, node_name):
    return get_topics(node_name, node.get_subscriber_names_and_types_by_node)


def get_publisher_info(node, node_name):
    return get_topics(node_name, node.get_publisher_names_and_types_by_node)


def get_service_info(node, node_name):
    return get_topics(node_name, node.get_service_names_and_types_by_node)


def find_openssl_executable():
    if platform.system() != 'Darwin':
        return 'openssl'

    brew_openssl_prefix_result = subprocess.run(
        ['brew', '--prefix', 'openssl'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    if brew_openssl_prefix_result.returncode:
        raise RuntimeError('unable to find openssl from brew')
    basepath = brew_openssl_prefix_result.stdout.decode().rstrip()
    return os.path.join(basepath, 'bin', 'openssl')


def check_openssl_version(openssl_executable):
    openssl_version_string_result = subprocess.run(
        [openssl_executable, 'version'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE
    )
    if openssl_version_string_result.returncode:
        raise RuntimeError('unable to invoke command: "%s"' % openssl_executable)
    version = openssl_version_string_result.stdout.decode().rstrip()
    openssl_version_string_list = version.split(' ')
    if openssl_version_string_list[0].lower() != 'openssl':
        raise RuntimeError(
            "expected version of the format 'OpenSSL "
            "<MAJOR>.<MINOR>.<PATCH_number><PATCH_letter>  <DATE>'")
    (major, minor, patch) = openssl_version_string_list[1].split('.')
    major = int(major)
    minor = int(minor)
    if major < 1:
        raise RuntimeError('need openssl 1.0.2 minimum')
    if major == 1 and minor < 0:
        raise RuntimeError('need openssl 1.0.2 minimum')
    if major == 1 and minor == 0 and int(''.join(itertools.takewhile(str.isdigit, patch))) < 2:
        raise RuntimeError('need openssl 1.0.2 minimum')


def create_ca_conf_file(path):
    with open(path, 'w') as f:
        f.write("""\
[ ca ]
default_ca = CA_default

[ CA_default ]
dir = .
certs = $dir/certs
crl_dir = $dir/crl
database = $dir/index.txt
unique_subject = no
new_certs_dir = $dir
certificate = $dir/ca.cert.pem
private_key = $dir/ca.key.pem
serial = $dir/serial
crlnumber = $dir/crlnumber
crl = $dir/crl.pem
RANDFILE = $dir/private/.rand
name_opt = ca_default
cert_opt = ca_default
default_days = 1825
default_crl_days = 30
default_md = sha256
preserve = no
policy = policy_match
x509_extensions = local_ca_extensions
#
#
# Copy extensions specified in the certificate request
#
copy_extensions = copy

[ policy_match ]
countryName = optional
stateOrProvinceName = optional
organizationName = optional
organizationalUnitName = optional
commonName = supplied
emailAddress = optional

#
#
# x509 extensions to use when generating server certificates.
#
[ local_ca_extensions ]
basicConstraints = CA:false

[ req ]
prompt = no
distinguished_name = req_distinguished_name
string_mask = utf8only
x509_extensions = root_ca_extensions

[ req_distinguished_name ]
commonName = sros2testCA

[ root_ca_extensions ]
basicConstraints = CA:true
""")


def run_shell_command(cmd, in_path=None):
    print('running command in path [%s]: %s' % (in_path, cmd))
    subprocess.call(cmd, shell=True, cwd=in_path)


def create_ecdsa_param_file(path):
    openssl_executable = find_openssl_executable()
    check_openssl_version(openssl_executable)
    run_shell_command('%s ecparam -name prime256v1 > %s' % (openssl_executable, path))


def create_ca_key_cert(ecdsa_param_path, ca_conf_path, ca_key_path, ca_cert_path):
    openssl_executable = find_openssl_executable()
    check_openssl_version(openssl_executable)
    run_shell_command(
        '%s req -nodes -x509 -days 3650 -newkey ec:%s -keyout %s -out %s -config %s' %
        (openssl_executable, ecdsa_param_path, ca_key_path, ca_cert_path, ca_conf_path))


def create_governance_file(path, domain_id):
    # for this application we are only looking to authenticate and encrypt;
    # we do not need/want access control at this point.
    governance_xml_path = get_transport_default('dds', 'governance.xml')
    governance_xml = etree.parse(governance_xml_path)

    governance_xsd_path = get_transport_schema('dds', 'governance.xsd')
    governance_xsd = etree.XMLSchema(etree.parse(governance_xsd_path))

    domain_id_elements = governance_xml.findall(
        'domain_access_rules/domain_rule/domains/id')
    for domain_id_element in domain_id_elements:
        domain_id_element.text = domain_id

    try:
        governance_xsd.assertValid(governance_xml)
    except etree.DocumentInvalid as e:
        raise RuntimeError(str(e))

    with open(path, 'wb') as f:
        f.write(etree.tostring(governance_xml, pretty_print=True))


def create_signed_governance_file(signed_gov_path, gov_path, ca_cert_path, ca_key_path):
    openssl_executable = find_openssl_executable()
    check_openssl_version(openssl_executable)
    run_shell_command(
        '%s smime -sign -in %s -text -out %s -signer %s -inkey %s' %
        (openssl_executable, gov_path, signed_gov_path, ca_cert_path, ca_key_path))


def create_keystore(keystore_path):
    if not os.path.exists(keystore_path):
        print('creating directory: %s' % keystore_path)
        os.makedirs(keystore_path, exist_ok=True)
    else:
        print('directory already exists: %s' % keystore_path)

    ca_conf_path = os.path.join(keystore_path, 'ca_conf.cnf')
    if not os.path.isfile(ca_conf_path):
        print('creating CA file: %s' % ca_conf_path)
        create_ca_conf_file(ca_conf_path)
    else:
        print('found CA conf file, not writing a new one!')

    ecdsa_param_path = os.path.join(keystore_path, 'ecdsaparam')
    if not os.path.isfile(ecdsa_param_path):
        print('creating ECDSA param file: %s' % ecdsa_param_path)
        create_ecdsa_param_file(ecdsa_param_path)
    else:
        print('found ECDSA param file, not writing a new one!')

    ca_key_path = os.path.join(keystore_path, 'ca.key.pem')
    ca_cert_path = os.path.join(keystore_path, 'ca.cert.pem')
    if not (os.path.isfile(ca_key_path) and os.path.isfile(ca_cert_path)):
        print('creating new CA key/cert pair')
        create_ca_key_cert(ecdsa_param_path, ca_conf_path, ca_key_path, ca_cert_path)
    else:
        print('found CA key and cert, not creating new ones!')

    # create governance file
    gov_path = os.path.join(keystore_path, 'governance.xml')
    if not os.path.isfile(gov_path):
        print('creating governance file: %s' % gov_path)
        domain_id = os.getenv(DOMAIN_ID_ENV, '0')
        create_governance_file(gov_path, domain_id)
    else:
        print('found governance file, not creating a new one!')

    # sign governance file
    signed_gov_path = os.path.join(keystore_path, 'governance.p7s')
    if not os.path.isfile(signed_gov_path):
        print('creating signed governance file: %s' % signed_gov_path)
        create_signed_governance_file(signed_gov_path, gov_path, ca_cert_path, ca_key_path)
    else:
        print('found signed governance file, not creating a new one!')

    # create index file
    index_path = os.path.join(keystore_path, 'index.txt')
    if not os.path.isfile(index_path):
        with open(index_path, 'a'):
            pass

    # create serial file
    serial_path = os.path.join(keystore_path, 'serial')
    if not os.path.isfile(serial_path):
        with open(serial_path, 'w') as f:
            f.write('1000')

    print('all done! enjoy your keystore in %s' % keystore_path)
    print('cheers!')
    return True


def is_valid_keystore(path):
    res = os.path.isfile(os.path.join(path, 'ca_conf.cnf'))
    res &= os.path.isfile(os.path.join(path, 'ecdsaparam'))
    res &= os.path.isfile(os.path.join(path, 'index.txt'))
    res &= os.path.isfile(os.path.join(path, 'ca.key.pem'))
    res &= os.path.isfile(os.path.join(path, 'ca.cert.pem'))
    res &= os.path.isfile(os.path.join(path, 'governance.p7s'))
    return res


def is_key_name_valid(name):
    ns_and_name = name.rsplit('/', 1)
    if len(ns_and_name) != 2:
        print("The key name needs to start with '/'")
        return False
    node_ns = ns_and_name[0] if ns_and_name[0] else '/'
    node_name = ns_and_name[1]

    try:
        return (validate_namespace(node_ns) and validate_node_name(node_name))
    except (InvalidNamespaceException, InvalidNodeNameException) as e:
        print('{}'.format(e))
        return False


def create_request_file(path, name):
    with open(path, 'w') as f:
        f.write("""\
prompt = no
string_mask = utf8only
distinguished_name = req_distinguished_name

[ req_distinguished_name ]
commonName = %s
""" % name)


def create_key_and_cert_req(root, relative_path, cnf_path, ecdsa_param_path, key_path, req_path):
    key_relpath = os.path.join(relative_path, 'key.pem')
    ecdsa_param_relpath = os.path.join(relative_path, 'ecdsaparam')
    cnf_relpath = os.path.join(relative_path, 'request.cnf')
    key_relpath = os.path.join(relative_path, 'key.pem')
    req_relpath = os.path.join(relative_path, 'req.pem')
    openssl_executable = find_openssl_executable()
    check_openssl_version(openssl_executable)
    run_shell_command(
        '%s req -nodes -new -newkey ec:%s -config %s -keyout %s -out %s' %
        (openssl_executable, ecdsa_param_relpath, cnf_relpath, key_relpath, req_relpath), root)


def create_cert(root_path, relative_path):
    req_relpath = os.path.join(relative_path, 'req.pem')
    cert_relpath = os.path.join(relative_path, 'cert.pem')
    openssl_executable = find_openssl_executable()
    check_openssl_version(openssl_executable)
    run_shell_command(
        '%s ca -batch -create_serial -config ca_conf.cnf -days 3650 -in %s -out %s' %
        (openssl_executable, req_relpath, cert_relpath), root_path)


def create_permission_file(path, domain_id, policy_element):

    permissions_xsl_path = get_transport_template('dds', 'permissions.xsl')
    permissions_xsl = etree.XSLT(etree.parse(permissions_xsl_path))
    permissions_xsd_path = get_transport_schema('dds', 'permissions.xsd')
    permissions_xsd = etree.XMLSchema(etree.parse(permissions_xsd_path))

    permissions_xml = permissions_xsl(policy_element)

    domain_id_elements = permissions_xml.findall('permissions/grant/*/domains/id')
    for domain_id_element in domain_id_elements:
        domain_id_element.text = domain_id

    try:
        permissions_xsd.assertValid(permissions_xml)
    except etree.DocumentInvalid as e:
        raise RuntimeError(str(e))

    with open(path, 'wb') as f:
        f.write(etree.tostring(permissions_xml, pretty_print=True))


def get_policy(name, policy_file_path):
    policy_tree = load_policy(policy_file_path)
    return get_policy_from_tree(name, policy_tree)


def get_policy_from_tree(name, policy_tree):
    ns, node = name.rsplit('/', 1)
    ns = '/' if not ns else ns
    profile_element = policy_tree.find(
        path='profiles/profile[@ns="{ns}"][@node="{node}"]'.format(
            ns=ns,
            node=node))
    if profile_element is None:
        raise RuntimeError('unable to find profile "{name}"'.format(
            name=name
        ))
    profiles_element = etree.Element('profiles')
    profiles_element.append(profile_element)
    policy_element = etree.Element('policy')
    policy_element.append(profiles_element)
    return policy_element


def create_signed_permissions_file(
        permissions_path, signed_permissions_path, ca_cert_path, ca_key_path):

    openssl_executable = find_openssl_executable()
    check_openssl_version(openssl_executable)
    run_shell_command(
        '%s smime -sign -in %s -text -out %s -signer %s -inkey %s' %
        (openssl_executable, permissions_path, signed_permissions_path, ca_cert_path, ca_key_path))


def create_permission(keystore_path, identity, policy_file_path):
    policy_element = get_policy(identity, policy_file_path)
    create_permissions_from_policy_element(keystore_path, identity, policy_element)
    return True


def create_permissions_from_policy_element(keystore_path, identity, policy_element):
    domain_id = os.getenv(DOMAIN_ID_ENV, '0')
    relative_path = os.path.normpath(identity.lstrip('/'))
    key_dir = os.path.join(keystore_path, relative_path)
    print('key_dir %s' % key_dir)
    permissions_path = os.path.join(key_dir, 'permissions.xml')
    create_permission_file(permissions_path, domain_id, policy_element)

    signed_permissions_path = os.path.join(key_dir, 'permissions.p7s')
    keystore_ca_cert_path = os.path.join(keystore_path, 'ca.cert.pem')
    keystore_ca_key_path = os.path.join(keystore_path, 'ca.key.pem')
    create_signed_permissions_file(
        permissions_path, signed_permissions_path,
        keystore_ca_cert_path, keystore_ca_key_path)


def create_key(keystore_path, identity):
    if not is_valid_keystore(keystore_path):
        print("'%s' is not a valid keystore " % keystore_path)
        return False
    if not is_key_name_valid(identity):
        return False
    print("creating key for identity: '%s'" % identity)

    relative_path = os.path.normpath(identity.lstrip('/'))
    key_dir = os.path.join(keystore_path, relative_path)
    os.makedirs(key_dir, exist_ok=True)

    # copy the CA cert in there
    keystore_ca_cert_path = os.path.join(keystore_path, 'ca.cert.pem')
    dest_identity_ca_cert_path = os.path.join(key_dir, 'identity_ca.cert.pem')
    dest_permissions_ca_cert_path = os.path.join(key_dir, 'permissions_ca.cert.pem')
    shutil.copyfile(keystore_ca_cert_path, dest_identity_ca_cert_path)
    shutil.copyfile(keystore_ca_cert_path, dest_permissions_ca_cert_path)

    # copy the governance file in there
    keystore_governance_path = os.path.join(keystore_path, 'governance.p7s')
    dest_governance_path = os.path.join(key_dir, 'governance.p7s')
    shutil.copyfile(keystore_governance_path, dest_governance_path)

    ecdsa_param_path = os.path.join(key_dir, 'ecdsaparam')
    if not os.path.isfile(ecdsa_param_path):
        print('creating ECDSA param file: %s' % ecdsa_param_path)
        create_ecdsa_param_file(ecdsa_param_path)
    else:
        print('found ECDSA param file, not writing a new one!')

    cnf_path = os.path.join(key_dir, 'request.cnf')
    if not os.path.isfile(cnf_path):
        create_request_file(cnf_path, identity)
    else:
        print('config file exists, not creating a new one: %s' % cnf_path)

    key_path = os.path.join(key_dir, 'key.pem')
    req_path = os.path.join(key_dir, 'req.pem')
    if not os.path.isfile(key_path) or not os.path.isfile(req_path):
        print('creating key and cert request')
        create_key_and_cert_req(
            keystore_path,
            relative_path,
            cnf_path,
            ecdsa_param_path,
            key_path, req_path)
    else:
        print('found key and cert req; not creating new ones!')

    cert_path = os.path.join(key_dir, 'cert.pem')
    if not os.path.isfile(cert_path):
        print('creating cert')
        create_cert(keystore_path, relative_path)
    else:
        print('found cert; not creating a new one!')

    # create a wildcard permissions file for this node which can be overridden
    # later using a policy if desired
    policy_file_path = get_policy_default('policy.xml')
    policy_element = get_policy('/default', policy_file_path)
    profile_element = policy_element.find('profiles/profile')
    ns, node = identity.rsplit('/', 1)
    ns = '/' if not ns else ns
    profile_element.attrib['ns'] = ns
    profile_element.attrib['node'] = node

    permissions_path = os.path.join(key_dir, 'permissions.xml')
    domain_id = os.getenv(DOMAIN_ID_ENV, '0')
    create_permission_file(permissions_path, domain_id, policy_element)

    signed_permissions_path = os.path.join(key_dir, 'permissions.p7s')
    keystore_ca_key_path = os.path.join(keystore_path, 'ca.key.pem')
    create_signed_permissions_file(
        permissions_path, signed_permissions_path,
        keystore_ca_cert_path, keystore_ca_key_path)

    return True


def list_keys(keystore_path):
    for name in os.listdir(keystore_path):
        if os.path.isdir(os.path.join(keystore_path, name)):
            print(name)
    return True


def distribute_key(source_keystore_path, taget_keystore_path):
    raise NotImplementedError()


def get_keystore_path_from_env():
    root_keystore_env_var = 'ROS_SECURITY_ROOT_DIRECTORY'
    root_keystore_path = os.getenv(root_keystore_env_var)
    if root_keystore_path is None:
        print('%s is empty' % root_keystore_env_var, file=sys.stderr)
    return root_keystore_path


def generate_artifacts(keystore_path=None, identity_names=[], policy_files=[]):
    if keystore_path is None:
        keystore_path = get_keystore_path_from_env()
        if keystore_path is None:
            return False
    if not is_valid_keystore(keystore_path):
        print('%s is not a valid keystore, creating new keystore' % keystore_path)
        create_keystore(keystore_path)

    # create keys for all provided identities
    for identity in identity_names:
        if not create_key(keystore_path, identity):
            return False
    for policy_file in policy_files:
        policy_tree = load_policy(policy_file)
        profiles_element = policy_tree.find('profiles')
        for profile in profiles_element:
            identity_name = profile.get('ns').rstrip('/') + '/' + profile.get('node')
            if not create_key(keystore_path, identity_name):
                return False
            policy_element = get_policy_from_tree(identity_name, policy_tree)
            create_permissions_from_policy_element(
                keystore_path, identity_name, policy_element)
    return True
