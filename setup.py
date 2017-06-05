from setuptools import setup

setup(
    name='sros2',
    version='0.0.0',
    packages=[],
    py_modules=['scripts.sros2'],
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
            'sros2 = scripts.sros2:main',
        ],
    }
)
