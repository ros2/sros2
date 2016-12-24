# Copyright 2016 Open Source Robotics Foundation, Inc.
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
private_key      = $dir/ca.key.pem
RANDFILE         = $dir/private/.rand
name_opt         = ca_default
cert_opt         = ca_default
default_days     = 1825
default_crl_days = 30
default_md       = sha256
preserve         = no
policy           = policy_match

[ policy_match ]
countryName = match
stateOrPrivinceName = match
organizationName = match
organizationalUnitName = optional
commonName = supplied
emailAddress = optional

[ policy_anything ]
countryName = optional
stateOrPrivinceName = optional
organizationName = optional
organizationalUnitName = optional
commonName = supplied
emailAddress = optional

[ req ]
prompt = no
distinguished_name = req_distinguished_name
string_mask = utf8only

[ req_distinguished_name ]
commonName = sros2testCA

""")

def run_shell_command(cmd):
    print("running command: %s" % cmd)
    subprocess.call(cmd, shell=True)

def create_ecdsa_param_file(path):
    run_shell_command("openssl ecparam -name prime256v1 > %s" % path)

def create_ca_key_cert(ecdsa_param_path, ca_conf_path, ca_key_path, ca_cert_path):
    run_shell_command("openssl req -nodes -x509 -days 3650 -newkey ec:%s -keyout %s -out %s -config %s" % (ecdsa_param_path, ca_key_path, ca_cert_path, ca_conf_path))

def create_governance_file(path):
    # for this application we are only looking to authenticate and encrypt;
    # we do not need/want access control at this point.
    with open(path, 'w') as f:
        f.write("""\
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
    xsi:noNamespaceSchemaLocation="http://www.omg.org/spec/DDS-SECURITY/20140301/dds_security_governance.xsd">
    <domain_access_rules>
        <domain_rule>
            <domain_id>*</domain_id>
            <allow_unauthenticated_join>FALSE</allow_unauthenticated_join>
            <enable_join_access_control>TRUE</enable_join_access_control>
            <discovery_protection_kind>ENCRYPT</discovery_protection_kind>
            <liveliness_protection_kind>ENCRYPT</liveliness_protection_kind>
            <rtps_protection_kind>SIGN</rtps_protection_kind>
            <topic_access_rules>
                <topic_rule>
                    <topic_expression>*</topic_expression>
                    <enable_discovery_protection>TRUE</enable_discovery_protection>
                    <enable_read_access_control>FALSE</enable_read_access_control>
                    <enable_write_access_control>FALSE</enable_write_access_control>
                    <metadata_protection_kind>ENCRYPT</metadata_protection_kind>
                    <data_protection_kind>ENCRYPT</data_protection_kind>
                </topic_rule>
            </topic_access_rules>
        </domain_rule>
    </domain_access_rules>
</dds>
""")

def create_signed_governance_file(signed_gov_path, gov_path, ca_cert_path, ca_key_path):
    run_shell_command("openssl smime -sign -in %s -text -out %s -signer %s -inkey %s" % (gov_path, signed_gov_path, ca_cert_path, ca_key_path))

def create_keystore(args):
    root = args.ROOT
    print(args)

    if not os.path.exists(root):
        print("creating directory: %s" % root)
        os.makedirs(root)
    else:
        print("directory already exists: %s" % root)

    ca_conf_path = os.path.join(root, 'ca_conf.txt')
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
    if not os.path.isfile(gov_path):
        print("creating governance file: %s" % gov_path)
        create_governance_file(gov_path)
    else:
        print("found governance file, not creating a new one!")

    # sign governance file
    signed_gov_path = os.path.join(root, 'governance.p7s')
    if not os.path.isfile(signed_gov_path):
        print("creating signed governance file: %s" % signed_gov_path)
        create_signed_governance_file(signed_gov_path, gov_path, ca_cert_path, ca_key_path)
    else:
        print("found signed governance file, not creating a new one!")
    return True

def is_valid_keystore(path):
    ecdsa_param_found = os.path.isfile(os.path.join(path, 'ecdsaparam'))
    ca_key_found = os.path.isfile(os.path.join(path, 'ca.key.pem'))
    ca_cert_found = os.path.isfile(os.path.join(path, 'ca.cert.pem'))
    signed_gov_found = os.path.isfile(os.path.join(path, 'governance.p7s'))
    return ecdsa_param_found and ca_key_found and \
        ca_cert_found and signed_gov_found

def create_key(args):
    print(args)
    root = args.ROOT
    name = args.NAME
    if not is_valid_keystore(root):
        print("root path is not a valid keystore: %s" % root)
        return False
    print("creating key for node name: %s" % name)
    return True

def distribute_key(args):
    print("distributing key")
    print(args)
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

    parser_distribute_keys = subparsers.add_parser('distribute_key')
    parser_distribute_keys.set_defaults(which='distribute_key')
    parser_distribute_keys.add_argument('ROOT', help='root path of keystore')
    parser_distribute_keys.add_argument('TARGET', help='target keystore path')

    args = parser.parse_args(sysargs)

    if '-h' in sysargs or '--help' in sysargs:
        sys.exit(0)  # we're already done

    if not 'which' in args:
        parser.print_help()
        sys.exit("Error: No verb provided.")

    if args.which == 'create_keystore':
        create_keystore(args)
    elif args.which == 'create_key':
        create_key(args)
    elif args.which == 'distribute_key':
        distribute_key(args)
    else:
        parser.print_help()
        sys.exit("Error: Unknown verb '{0}' provided.".format(args['which']))

    sys.exit(0)

if __name__ == '__main__':
    main()
