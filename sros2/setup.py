import glob
import os

from setuptools import find_packages
from setuptools import setup


def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', path, filename))
    return paths


extra_files = (
    package_files('sros2/policy/defaults') +
    package_files('sros2/policy/schemas') +
    package_files('sros2/policy/templates')
)

package_name = 'sros2'

setup(
    name=package_name,
    version='0.13.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/sros2/xml_cache', glob.glob('xml_cache/**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Morgan Quigley',
    author_email='morgan@osrfoundation.org',
    maintainer='Mikael Arguedas',
    maintainer_email='mikael@osrfoundation.org',
    url='https://github.com/ros2/sros2',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='SROS2 provides tools to help manage security keys.',
    long_description="""\
SROS2 provides command-line tools to help generate keys and \
certificates which are then used by supported middleware implementations to \
enhance the security of ROS 2 deployments.""",
    license='Apache License, Version 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'ros2cli.command': [
            'security = sros2.command.security:SecurityCommand',
        ],
        'ros2cli.extension_point': [
            'sros2.verb = sros2.verb:VerbExtension',
        ],
        'sros2.verb': [
            # These are deprecated
            'create_key = sros2.verb.create_enclave:CreateKeyVerb',
            'list_keys = sros2.verb.list_enclaves:ListKeysVerb',

            'create_enclave = sros2.verb.create_enclave:CreateEnclaveVerb',
            'create_keystore = sros2.verb.create_keystore:CreateKeystoreVerb',
            'create_permission = sros2.verb.create_permission'
            ':CreatePermissionVerb',
            'generate_artifacts = sros2.verb.generate_artifacts:GenerateArtifactsVerb',
            'generate_policy = sros2.verb.generate_policy:GeneratePolicyVerb',
            'list_enclaves = sros2.verb.list_enclaves:ListEnclavesVerb',
        ],
    },
    package_data={
        'sros2': extra_files,
    },
)
