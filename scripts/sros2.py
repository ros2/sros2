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

import argparse
import os
import shutil
import subprocess
import sys


def create_ca_conf_file(path):
    with open(path, 'w') as f:
        f.write("""\
[ ca ]
default_ca = CA_default

[ CA_default ]
dir              = .
certs            = $dir/certs
crl_dir          = $dir/crl
database         = $dir/index.txt
unique_subject   = no
new_certs_dir    = $dir
certificate      = $dir/ca.cert.pem
private_key      = $dir/ca.key.pem
serial           = $dir/serial
crlnumber        = $dir/crlnumber
crl              = $dir/crl.pem
RANDFILE         = $dir/private/.rand
name_opt         = ca_default
cert_opt         = ca_default
default_days     = 1825
default_crl_days = 30
default_md       = sha256
preserve         = no
policy           = policy_match

[ policy_match ]
countryName            = optional
stateOrProvinceName    = optional
organizationName       = optional
organizationalUnitName = optional
commonName             = supplied
emailAddress           = optional

[ req ]
prompt = no
distinguished_name = req_distinguished_name
string_mask = utf8only

[ req_distinguished_name ]
commonName = sros2testCA

""")


def run_shell_command(cmd, in_path=None):
    print("running command in path [%s]: %s" % (in_path, cmd))
    subprocess.call(cmd, shell=True, cwd=in_path)


def create_ecdsa_param_file(path):
    run_shell_command("openssl ecparam -name prime256v1 > %s" % path)


def create_ca_key_cert(ecdsa_param_path, ca_conf_path, ca_key_path, ca_cert_path):
    run_shell_command(
        "openssl req -nodes -x509 -days 3650 -newkey ec:%s -keyout %s -out %s -config %s" %
        (ecdsa_param_path, ca_key_path, ca_cert_path, ca_conf_path))


def create_governance_file(path, domain_id):
    # for this application we are only looking to authenticate and encrypt;
    # we do not need/want access control at this point.
    with open(path, 'w') as f:
        f.write("""\
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://www.omg.org/spec/DDS-SECURITY/20140301/dds_security_governance.xsd">
    <domain_access_rules>
        <domain_rule>
            <domain_id>%s</domain_id>
            <allow_unauthenticated_join>FALSE</allow_unauthenticated_join>
            <enable_join_access_control>TRUE</enable_join_access_control>
            <discovery_protection_kind>ENCRYPT</discovery_protection_kind>
            <liveliness_protection_kind>ENCRYPT</liveliness_protection_kind>
            <rtps_protection_kind>SIGN</rtps_protection_kind>
            <topic_access_rules>
                <topic_rule>
                    <topic_expression>*</topic_expression>
                    <enable_discovery_protection>TRUE</enable_discovery_protection>
                    <enable_read_access_control>TRUE</enable_read_access_control>
                    <enable_write_access_control>TRUE</enable_write_access_control>
                    <metadata_protection_kind>ENCRYPT</metadata_protection_kind>
                    <data_protection_kind>ENCRYPT</data_protection_kind>
                </topic_rule>
            </topic_access_rules>
        </domain_rule>
    </domain_access_rules>
</dds>
""" % domain_id)


def create_signed_governance_file(signed_gov_path, gov_path, ca_cert_path, ca_key_path):
    run_shell_command(
        "openssl smime -sign -in %s -text -out %s -signer %s -inkey %s" %
        (gov_path, signed_gov_path, ca_cert_path, ca_key_path))


def create_keystore(args):
    root = args.ROOT
    print(args)

    if not os.path.exists(root):
        print("creating directory: %s" % root)
        os.makedirs(root)
    else:
        print("directory already exists: %s" % root)

    ca_conf_path = os.path.join(root, 'ca_conf.cnf')
    if not os.path.isfile(ca_conf_path):
        print("creating CA file: %s" % ca_conf_path)
        create_ca_conf_file(ca_conf_path)
    else:
        print("found CA conf file, not writing a new one!")

    ecdsa_param_path = os.path.join(root, 'ecdsaparam')
    if not os.path.isfile(ecdsa_param_path):
        print("creating ECDSA param file: %s" % ecdsa_param_path)
        create_ecdsa_param_file(ecdsa_param_path)
    else:
        print("found ECDSA param file, not writing a new one!")

    ca_key_path = os.path.join(root, 'ca.key.pem')
    ca_cert_path = os.path.join(root, 'ca.cert.pem')
    if not (os.path.isfile(ca_key_path) and os.path.isfile(ca_cert_path)):
        print("creating new CA key/cert pair")
        create_ca_key_cert(ecdsa_param_path, ca_conf_path, ca_key_path, ca_cert_path)
    else:
        print("found CA key and cert, not creating new ones!")

    # create governance file
    gov_path = os.path.join(root, 'governance.xml')
    domain_id = os.getenv('ROS_DOMAIN_ID', 0)
    if not os.path.isfile(gov_path):
        print("creating governance file: %s" % gov_path)
        create_governance_file(gov_path, domain_id)
    else:
        print("found governance file, not creating a new one!")

    # sign governance file
    signed_gov_path = os.path.join(root, 'governance.p7s')
    if not os.path.isfile(signed_gov_path):
        print("creating signed governance file: %s" % signed_gov_path)
        create_signed_governance_file(signed_gov_path, gov_path, ca_cert_path, ca_key_path)
    else:
        print("found signed governance file, not creating a new one!")

    # create index file
    index_path = os.path.join(root, 'index.txt')
    if not os.path.isfile(index_path):
        open(index_path, 'a').close()

    # create serial file
    serial_path = os.path.join(root, 'serial')
    if not os.path.isfile(serial_path):
        with open(serial_path, 'w') as f:
            f.write("1000")

    print("all done! enjoy your keystore in %s" % root)
    print("cheers!")
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
    if ('..' in name) or ('/' in name) or ('\\' in name):
        return False
    return True


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
    run_shell_command(
        "openssl req -nodes -new -newkey ec:%s -config %s -keyout %s -out %s" %
        (ecdsa_param_relpath, cnf_relpath, key_relpath, req_relpath), root)


def create_cert(root_path, name):
    req_relpath = os.path.join(name, "req.pem")
    cert_relpath = os.path.join(name, "cert.pem")
    run_shell_command(
        "openssl ca -batch -create_serial -config ca_conf.cnf -days 3650 -in %s -out %s" %
        (req_relpath, cert_relpath), root_path)


def create_permission_file(path, name, domain_id, permissions_dict):
    permission_str = """\
<permissions xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://www.omg.org/spec/DDS-SECURITY/20140301/dds_security_permissions.xsd">
  <grant name="%s_policies">
    <subject_name>CN=%s</subject_name>
    <validity>
      <!-- Format is YYYYMMDDHH in GMT -->
      <not_before>2016122000</not_before>
      <not_after>2026122000</not_after>
    </validity>
    <allow_rule>
      <domain_id>%s</domain_id>
""" % (name, name, domain_id)
    # access control only on topics for now
    topic_dict = permissions_dict['topics']
    if topic_dict is not None and topic_dict != {}:
        # we have some policies to add !
        for topic_name, policy in topic_dict.items():
            if policy['allow'] == 's':
                tag = 'subscribe'
            elif policy['allow'] == 'p':
                tag = 'publish'
            else:
                print("unknown permission policy '%s', skipping" % policy['allow'])
                continue
            permission_str += """\
      <%s>
        <topic>%s</topic>
      </%s>
""" % (tag, topic_name, tag)
        # DCPS* is necessary for builtin data readers
        permission_str += """\
      <subscribe>
        <topic>DCPS*</topic>
      </subscribe>
"""
    else:
        # no policy found: allow everything!
        permission_str += """\
      <publish>
        <topic>*</topic>
      </publish>
      <subscribe>
        <topic>*</topic>
      </subscribe>
"""

    permission_str += """\
    </allow_rule>
    <default>DENY</default>
  </grant>
</permissions>
"""
    with open(path, 'w') as f:
        f.write(permission_str)


def get_permissions(name, policy_file_path):
    import yaml
    if not os.path.isfile(policy_file_path):
        return {'topics': {}}
    with open(policy_file_path, 'r') as graph_permissions_file:
        try:
            graph = yaml.load(graph_permissions_file)
        except yaml.YAMLError as e:
            print(e)
            sys.exit(1)
        return graph['nodes'][name]


def create_signed_permissions_file(
        permissions_path, signed_permissions_path, ca_cert_path, ca_key_path):
    run_shell_command(
        "openssl smime -sign -in %s -text -out %s -signer %s -inkey %s" %
        (permissions_path, signed_permissions_path, ca_cert_path, ca_key_path))


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


def create_key(args):
    print(args)
    root = args.ROOT
    name = args.NAME
    if not is_valid_keystore(root):
        print("root path is not a valid keystore: %s" % root)
        return False
    if not is_key_name_valid(name):
        print("bad character in requested key name: %s" % name)
        return False
    print("creating key for node name: %s" % name)

    key_dir = os.path.join(root, name)
    if not os.path.exists(key_dir):
        os.makedirs(key_dir)

    # copy the CA cert in there
    keystore_ca_cert_path = os.path.join(root, 'ca.cert.pem')
    dest_ca_cert_path = os.path.join(key_dir, 'ca.cert.pem')
    shutil.copyfile(keystore_ca_cert_path, dest_ca_cert_path)

    # copy the governance file in there
    keystore_governance_path = os.path.join(root, 'governance.p7s')
    dest_governance_path = os.path.join(key_dir, 'governance.p7s')
    shutil.copyfile(keystore_governance_path, dest_governance_path)

    ecdsa_param_path = os.path.join(key_dir, 'ecdsaparam')
    if not os.path.isfile(ecdsa_param_path):
        print("creating ECDSA param file: %s" % ecdsa_param_path)
        create_ecdsa_param_file(ecdsa_param_path)
    else:
        print("found ECDSA param file, not writing a new one!")

    cnf_path = os.path.join(key_dir, 'request.cnf')
    if not os.path.isfile(cnf_path):
        create_request_file(cnf_path, name)
    else:
        print("config file exists, not creating a new one: %s" % cnf_path)

    key_path = os.path.join(key_dir, 'key.pem')
    req_path = os.path.join(key_dir, 'req.pem')
    if not os.path.isfile(key_path) or not os.path.isfile(req_path):
        print("creating key and cert request")
        create_key_and_cert_req(root, name, cnf_path, ecdsa_param_path, key_path, req_path)
    else:
        print("found key and cert req; not creating new ones!")

    cert_path = os.path.join(key_dir, 'cert.pem')
    if not os.path.isfile(cert_path):
        print("creating cert")
        create_cert(root, name)
    else:
        print("found cert; not creating a new one!")

    # create a wildcard permissions file for this node which can be overridden
    # later using a policy if desired
    domain_id = os.getenv('ROS_DOMAIN_ID', 0)
    permissions_path = os.path.join(key_dir, 'permissions.xml')
    create_permission_file(permissions_path, name, domain_id, {"topics": None})

    signed_permissions_path = os.path.join(key_dir, 'permissions.p7s')
    keystore_ca_key_path = os.path.join(root, 'ca.key.pem')
    create_signed_permissions_file(
        permissions_path, signed_permissions_path,
        keystore_ca_cert_path, keystore_ca_key_path)

    return True


def list_keys(args):
    for root, dirs, files in os.walk(args.ROOT):
        if root == args.ROOT:
            for d in dirs:
                print("%s" % d)
    return True


def distribute_key(args):
    print("distributing key")
    print(args)
    print("just kidding, sorry, this isn't implemented yet.")
    return True


def main(sysargs=None):
    sysargs = sys.argv[1:] if sysargs is None else sysargs

    parser = argparse.ArgumentParser(prog='sros2')
    subparsers = parser.add_subparsers()

    parser_create_keystore = subparsers.add_parser('create_keystore')
    parser_create_keystore.set_defaults(which='create_keystore')
    parser_create_keystore.add_argument('ROOT', help='root path of keystore')

    parser_create_key = subparsers.add_parser('create_key')
    parser_create_key.set_defaults(which='create_key')
    parser_create_key.add_argument('ROOT', help='root path of keystore')
    parser_create_key.add_argument('NAME', help='key name, aka ROS node name')

    parser_list_keys = subparsers.add_parser('list_keys')
    parser_list_keys.set_defaults(which='list_keys')
    parser_list_keys.add_argument('ROOT', help='root path of keystore')

    parser_distribute_keys = subparsers.add_parser('distribute_key')
    parser_distribute_keys.set_defaults(which='distribute_key')
    parser_distribute_keys.add_argument('ROOT', help='root path of keystore')
    parser_distribute_keys.add_argument('TARGET', help='target keystore path')

    parser_create_perm = subparsers.add_parser('create_permission')
    parser_create_perm.set_defaults(which='create_permission')
    parser_create_perm.add_argument('ROOT', help='root path of keystore')
    parser_create_perm.add_argument('NAME', help='key name, aka ROS node name')
    parser_create_perm.add_argument(
        'POLICY_FILE_PATH', help='path of the permission yaml file')

    args = parser.parse_args(sysargs)

    if '-h' in sysargs or '--help' in sysargs:
        sys.exit(0)  # we're already done

    if 'which' not in args:
        parser.print_help()
        sys.exit("Error: No verb provided.")

    result = False

    if args.which == 'create_keystore':
        result = create_keystore(args)
    elif args.which == 'create_key':
        result = create_key(args)
    elif args.which == 'create_permission':
        result = create_permission(args)
    elif args.which == 'list_keys':
        result = list_keys(args)
    elif args.which == 'distribute_key':
        result = distribute_key(args)
    else:
        parser.print_help()
        sys.exit("Error: Unknown verb '{0}' provided.".format(args['which']))

    if (result):
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == '__main__':
    main()