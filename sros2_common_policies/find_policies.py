#!/usr/bin/env python3

import os

from ament_index_python import get_resource
from ament_index_python import get_resources
from ament_index_python import has_resource

from ament_index_python import get_package_share_directory

POLICIES_RESOURCE_TYPE = 'sros2_policies'

def get_package_names_with_policies():
    """Get the names of all packages that register policies in the ament index."""
    return list(get_resources(POLICIES_RESOURCE_TYPE).keys())

def get_package_policies(*, package_name=None):
    """
    Get all policies registered in the ament index for the given package.
    :param package_name: whose policies are to be retrieved.
    :return: a list of policy names.
    """
    if not has_resource(POLICIES_RESOURCE_TYPE, package_name):
        return []
    policies, _ = get_resource(POLICIES_RESOURCE_TYPE, package_name)
    return policies.split(';')

def get_registered_policies():
    """
    Get all policies registered in the ament index.
    :return: a list of (package name, policy file) tuples.
    """
    return [
        (package_name, get_package_policies(package_name=package_name))
        for package_name in get_package_names_with_policies()
    ]

if __name__ == "__main__":
    policies = get_registered_policies()

    for (package, policies) in get_registered_policies():
        print(package)
        share_dir = get_package_share_directory(package)
        for policy in policies:
            print("\t" + policy)
            print("\t" + os.path.join(share_dir, policy))


