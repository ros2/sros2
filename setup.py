from __future__ import print_function

import sys

from setuptools import find_packages
from setuptools import setup

if sys.version_info < (3, 5):
    print('sros2 requires Python 3.5 or higher.', file=sys.stderr)
    sys.exit(1)

setup(
    name='sros2',
    version='0.0.0',
    packages=[],
    py_modules=['sros2'],
    install_requires=['setuptools'],
    author='Morgan Quigley',
    author_email='morgan@osrfoundation.org',
    maintainer='Morgan Quigley',
    maintainer_email='morgan@osrfoundation.org',
    url='https://github.com/ros2/sros2',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='SROS2 provides tools to help manage security keys.',
    long_description="""\
SROS2 provides command-line tools to help generate and distribute keys and
certificates which are then used by supported middleware implementations to
enhance the security of ROS 2 deployments.""",
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'sros2 = sros2:main',
        ],
    }
)
