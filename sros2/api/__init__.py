# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

import itertools
import os
import platform
import shutil
import subprocess


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
    with open(path, 'w') as f:
        f.write("""\
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://www.omg.org/spec/DDS-SECURITY/20170901/omg_shared_ca_governance.xsd">
    <domain_access_rules>
        <domain_rule>
            <domains>
              <id>%s</id>
            </domains>
            <allow_unauthenticated_participants>false</allow_unauthenticated_participants>
            <enable_join_access_control>true</enable_join_access_control>
            <discovery_protection_kind>ENCRYPT</discovery_protection_kind>
            <liveliness_protection_kind>ENCRYPT</liveliness_protection_kind>
            <rtps_protection_kind>SIGN</rtps_protection_kind>
            <topic_access_rules>
                <topic_rule>
                    <topic_expression>*</topic_expression>
                    <enable_discovery_protection>true</enable_discovery_protection>
                    <enable_liveliness_protection>true</enable_liveliness_protection>
                    <enable_read_access_control>true</enable_read_access_control>
                    <enable_write_access_control>true</enable_write_access_control>
                    <metadata_protection_kind>ENCRYPT</metadata_protection_kind>
                    <data_protection_kind>ENCRYPT</data_protection_kind>
                </topic_rule>
            </topic_access_rules>
        </domain_rule>
    </domain_access_rules>
</dds>
""" % domain_id)


def create_signed_governance_file(signed_gov_path, gov_path, ca_cert_path, ca_key_path):
    openssl_executable = find_openssl_executable()
    check_openssl_version(openssl_executable)
    run_shell_command(
        '%s smime -sign -in %s -text -out %s -signer %s -inkey %s' %
        (openssl_executable, gov_path, signed_gov_path, ca_cert_path, ca_key_path))


def create_keystore(args):
    root = args.ROOT
    print(args)

    if not os.path.exists(root):
        print('creating directory: %s' % root)
        os.makedirs(root, exist_ok=True)
    else:
        print('directory already exists: %s' % root)

    ca_conf_path = os.path.join(root, 'ca_conf.cnf')
    if not os.path.isfile(ca_conf_path):
        print('creating CA file: %s' % ca_conf_path)
        create_ca_conf_file(ca_conf_path)
    else:
        print('found CA conf file, not writing a new one!')

    ecdsa_param_path = os.path.join(root, 'ecdsaparam')
    if not os.path.isfile(ecdsa_param_path):
        print('creating ECDSA param file: %s' % ecdsa_param_path)
        create_ecdsa_param_file(ecdsa_param_path)
    else:
        print('found ECDSA param file, not writing a new one!')

    ca_key_path = os.path.join(root, 'ca.key.pem')
    ca_cert_path = os.path.join(root, 'ca.cert.pem')
    if not (os.path.isfile(ca_key_path) and os.path.isfile(ca_cert_path)):
        print('creating new CA key/cert pair')
        create_ca_key_cert(ecdsa_param_path, ca_conf_path, ca_key_path, ca_cert_path)
    else:
        print('found CA key and cert, not creating new ones!')

    # create governance file
    gov_path = os.path.join(root, 'governance.xml')
    if not os.path.isfile(gov_path):
        print('creating governance file: %s' % gov_path)
        domain_id = os.getenv('ROS_DOMAIN_ID', 0)
        create_governance_file(gov_path, domain_id)
    else:
        print('found governance file, not creating a new one!')

    # sign governance file
    signed_gov_path = os.path.join(root, 'governance.p7s')
    if not os.path.isfile(signed_gov_path):
        print('creating signed governance file: %s' % signed_gov_path)
        create_signed_governance_file(signed_gov_path, gov_path, ca_cert_path, ca_key_path)
    else:
        print('found signed governance file, not creating a new one!')

    # create index file
    index_path = os.path.join(root, 'index.txt')
    if not os.path.isfile(index_path):
        with open(index_path, 'a'):
            pass

    # create serial file
    serial_path = os.path.join(root, 'serial')
    if not os.path.isfile(serial_path):
        with open(serial_path, 'w') as f:
            f.write('1000')

    print('all done! enjoy your keystore in %s' % root)
    print('cheers!')
    return True


def is_valid_keystore(path):
    ca_conf_found = os.path.isfile(os.path.join(path, 'ca_conf.cnf'))
    ecdsa_param_found = os.path.isfile(os.path.join(path, 'ecdsaparam'))
    index_found = os.path.isfile(os.path.join(path, 'index.txt'))
    ca_key_found = os.path.isfile(os.path.join(path, 'ca.key.pem'))
    ca_cert_found = os.path.isfile(os.path.join(path, 'ca.cert.pem'))
    signed_gov_found = os.path.isfile(os.path.join(path, 'governance.p7s'))
    return ecdsa_param_found and ca_key_found and \
        ca_cert_found and signed_gov_found and \
        index_found and ca_conf_found


def is_key_name_valid(name):
    # quick check for obvious filesystem problems
    return '..' not in name and '/' not in name and '\\' not in name


def create_request_file(path, name):
    with open(path, 'w') as f:
        f.write("""\
prompt = no
string_mask = utf8only
distinguished_name = req_distinguished_name

[ req_distinguished_name ]
commonName = %s
""" % name)


def create_key_and_cert_req(root, name, cnf_path, ecdsa_param_path, key_path, req_path):
    key_relpath = os.path.join(name, 'key.pem')
    ecdsa_param_relpath = os.path.join(name, 'ecdsaparam')
    cnf_relpath = os.path.join(name, 'request.cnf')
    key_relpath = os.path.join(name, 'key.pem')
    req_relpath = os.path.join(name, 'req.pem')
    openssl_executable = find_openssl_executable()
    check_openssl_version(openssl_executable)
    run_shell_command(
        '%s req -nodes -new -newkey ec:%s -config %s -keyout %s -out %s' %
        (openssl_executable, ecdsa_param_relpath, cnf_relpath, key_relpath, req_relpath), root)


def create_cert(root_path, name):
    req_relpath = os.path.join(name, 'req.pem')
    cert_relpath = os.path.join(name, 'cert.pem')
    openssl_executable = find_openssl_executable()
    check_openssl_version(openssl_executable)
    run_shell_command(
        '%s ca -batch -create_serial -config ca_conf.cnf -days 3650 -in %s -out %s' %
        (openssl_executable, req_relpath, cert_relpath), root_path)


def create_permission_file(path, name, domain_id, permissions_dict):
    permission_str = """\
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://www.omg.org/spec/DDS-SECURITY/20170901/omg_shared_ca_permissions.xsd">
  <permissions>
    <grant name="%s_policies">
      <subject_name>CN=%s</subject_name>
      <validity>
      <!--
       Format is CCYY-MM-DDThh:mm:ss[Z|(+|-)hh:mm]
                           The time zone may be specified as Z (UTC) or (+|-)hh:mm.
                           Time zones that aren't specified are considered UTC.
      -->
        <not_before>2013-10-26T00:00:00</not_before>
        <not_after>2023-10-26T22:45:30</not_after>
      </validity>
      <allow_rule>
        <domains>
          <id>%s</id>
        </domains>
""" % (name, name, domain_id)
    # access control only on topics for now
    topic_dict = permissions_dict['topics']
    if topic_dict:
        # add rules for automatically created ros2 topics
        # TODO(mikaelarguedas) remove this hardcoded handling for default topics
        # TODO(mikaelarguedas) update dictionary based on existing rule
        # if it already exists (rather than overriding the rule)
        topic_dict['parameter_events'] = {'allow': 'ps'}
        topic_dict['clock'] = {'allow': 's'}
        # we have some policies to add !
        for topic_name, policy in topic_dict.items():
            tags = []
            if policy['allow'] == 'ps':
                tags = ['publish', 'subscribe']
            elif policy['allow'] == 's':
                tags = ['subscribe']
            elif policy['allow'] == 'p':
                tags = ['publish']
            else:
                print("unknown permission policy '%s', skipping" % policy['allow'])
                continue
            for tag in tags:
                permission_str += """\
        <%s>
          <partitions>
            <partition></partition>
          </partitions>
          <topics>
            <topic>%s</topic>
          </topics>
        </%s>
""" % (tag, 'rt/' + topic_name, tag)
        # TODO(mikaelarguedas) remove this hardcoded handling for default parameter topics
        service_topic_prefixes = {
            'Request': 'rq/%s/' % name,
            'Reply': 'rr/%s/' % name,
        }
        default_parameter_topics = [
            'describe_parameters',
            'get_parameters',
            'get_parameter_types',
            'list_parameters',
            'set_parameters',
            'set_parameters_atomically',
        ]
        for topic_suffix, topic_prefix in service_topic_prefixes.items():
            service_topics = [
                (topic_prefix + topic + topic_suffix) for topic in default_parameter_topics]
            topics_string = ''
            for service_topic in service_topics:
                topics_string += """
            <topic>%s</topic>""" % (service_topic)
            permission_str += """
        <publish>
          <partitions>
            <partition></partition>
          </partitions>
          <topics>%s
          </topics>
        </publish>
        <subscribe>
          <partitions>
            <partition></partition>
          </partitions>
          <topics>%s
          </topics>
        </subscribe>
""" % (topics_string, topics_string)

    else:
        # no policy found: allow everything!
        permission_str += """\
        <publish>
          <partitions>
            <partition>*</partition>
          </partitions>
          <topics>
            <topic>*</topic>
          </topics>
        </publish>
        <subscribe>
          <partitions>
            <partition>*</partition>
          </partitions>
          <topics>
            <topic>*</topic>
          </topics>
        </subscribe>
"""

    permission_str += """\
      </allow_rule>
      <default>DENY</default>
    </grant>
  </permissions>
</dds>
"""
    with open(path, 'w') as f:
        f.write(permission_str)


def get_permissions(name, policy_file_path):
    import yaml
    if not os.path.isfile(policy_file_path):
        raise FileNotFoundError("policy file '%s' does not exist" % policy_file_path)
    with open(policy_file_path, 'r') as graph_permissions_file:
        try:
            graph = yaml.load(graph_permissions_file)
        except yaml.YAMLError as e:
            raise RuntimeError(str(e))
        return graph['nodes'][name]


def create_signed_permissions_file(
        permissions_path, signed_permissions_path, ca_cert_path, ca_key_path):

    openssl_executable = find_openssl_executable()
    check_openssl_version(openssl_executable)
    run_shell_command(
        '%s smime -sign -in %s -text -out %s -signer %s -inkey %s' %
        (openssl_executable, permissions_path, signed_permissions_path, ca_cert_path, ca_key_path))


def create_permission(args):
    print(args)
    root = args.ROOT
    name = args.NAME
    policy_file_path = args.POLICY_FILE_PATH
    domain_id = os.getenv('ROS_DOMAIN_ID', 0)

    key_dir = os.path.join(root, name)
    print('key_dir %s' % key_dir)
    permissions_dict = get_permissions(name, policy_file_path)
    permissions_path = os.path.join(key_dir, 'permissions.xml')
    create_permission_file(permissions_path, name, domain_id, permissions_dict)

    signed_permissions_path = os.path.join(key_dir, 'permissions.p7s')
    keystore_ca_cert_path = os.path.join(root, 'ca.cert.pem')
    keystore_ca_key_path = os.path.join(root, 'ca.key.pem')
    create_signed_permissions_file(
        permissions_path, signed_permissions_path,
        keystore_ca_cert_path, keystore_ca_key_path)
    return True


def create_key(args):
    print(args)
    root = args.ROOT
    name = args.NAME
    if not is_valid_keystore(root):
        print('root path is not a valid keystore: %s' % root)
        return False
    if not is_key_name_valid(name):
        print('bad character in requested key name: %s' % name)
        return False
    print('creating key for node name: %s' % name)

    key_dir = os.path.join(root, name)
    os.makedirs(key_dir, exist_ok=True)

    # copy the CA cert in there
    keystore_ca_cert_path = os.path.join(root, 'ca.cert.pem')
    dest_identity_ca_cert_path = os.path.join(key_dir, 'identity_ca.cert.pem')
    dest_permissions_ca_cert_path = os.path.join(key_dir, 'permissions_ca.cert.pem')
    shutil.copyfile(keystore_ca_cert_path, dest_identity_ca_cert_path)
    shutil.copyfile(keystore_ca_cert_path, dest_permissions_ca_cert_path)

    # copy the governance file in there
    keystore_governance_path = os.path.join(root, 'governance.p7s')
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
        create_request_file(cnf_path, name)
    else:
        print('config file exists, not creating a new one: %s' % cnf_path)

    key_path = os.path.join(key_dir, 'key.pem')
    req_path = os.path.join(key_dir, 'req.pem')
    if not os.path.isfile(key_path) or not os.path.isfile(req_path):
        print('creating key and cert request')
        create_key_and_cert_req(root, name, cnf_path, ecdsa_param_path, key_path, req_path)
    else:
        print('found key and cert req; not creating new ones!')

    cert_path = os.path.join(key_dir, 'cert.pem')
    if not os.path.isfile(cert_path):
        print('creating cert')
        create_cert(root, name)
    else:
        print('found cert; not creating a new one!')

    # create a wildcard permissions file for this node which can be overridden
    # later using a policy if desired
    domain_id = os.getenv('ROS_DOMAIN_ID', 0)
    permissions_path = os.path.join(key_dir, 'permissions.xml')
    create_permission_file(permissions_path, name, domain_id, {'topics': None})

    signed_permissions_path = os.path.join(key_dir, 'permissions.p7s')
    keystore_ca_key_path = os.path.join(root, 'ca.key.pem')
    create_signed_permissions_file(
        permissions_path, signed_permissions_path,
        keystore_ca_cert_path, keystore_ca_key_path)

    return True


def list_keys(args):
    for name in os.listdir(args.ROOT):
        if os.path.isdir(os.path.join(args.ROOT, name)):
            print(name)
    return True


def distribute_key(args):
    raise NotImplementedError()
