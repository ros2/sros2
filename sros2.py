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
import sys

def create_keystore(args):
    print("creating keystore")
    print(args)

def distribute_keys(args):
    print("distributing keys")
    print(args)

def main(sysargs=None):
    sysargs = sys.argv[1:] if sysargs is None else sysargs

    parser = argparse.ArgumentParser(prog='sros2')
    subparsers = parser.add_subparsers()

    parser_create_keystore = subparsers.add_parser('create_keystore')
    parser_create_keystore.add_argument('ROOT', help='root path of keystore')
    parser_create_keystore.set_defaults(which='create_keystore')

    parser_distribute_keys = subparsers.add_parser('distribute_keys')
    parser_distribute_keys.add_argument('ROOT', help='root path of keystore')
    parser_distribute_keys.add_argument('TARGET', help='target keystore path')
    parser_distribute_keys.set_defaults(which='distribute_keys')

    args = parser.parse_args(sysargs)

    if '-h' in sysargs or '--help' in sysargs:
        sys.exit(0)  # we're already done

    if not 'which' in args:
        parser.print_help()
        sys.exit("Error: No verb provided.")

    if args.which == 'create_keystore':
        create_keystore(args)
    elif args.which == 'distribute_keys':
        distribute_keys(args)
    else:
        parser.print_help()
        sys.exit("Error: Unknown verb '{0}' provided.".format(args['which']))

    sys.exit(0)

if __name__ == '__main__':
    main()
