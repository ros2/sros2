# Context

[![SROS2 CI](https://github.com/ros2/sros2/workflows/SROS2%20CI/badge.svg)](https://github.com/ros2/sros2/actions?query=workflow%3A%22SROS2+CI%22+branch%3Amaster)
[![codecov](https://codecov.io/gh/ros2/sros2/branch/master/graph/badge.svg)](https://codecov.io/gh/ros2/sros2)

This package provides the tools and instructions to use ROS2 on top of DDS-Security.
The security feature is tested across platforms (Linux, macOS, and Windows) as well as across different languages (C++ and Python).

Although we are designing SROS2 to work with any secure middleware, at the moment we are testing with RTI Connext Secure 5.3.1 and eProsima's Fast-RTPS 1.6.0.
If you want to run the demo using RTI Connext Secure you will need a license for it and you will need to install it.

These Tutorials are written for the latest state of the repository.
If you are using an older ROS 2 distribution please refer to the tutorials on the branch named after the distribution, e.g. for Crystal: https://github.com/ros2/sros2/blob/crystal/README.md

[Try SROS2 on Linux](SROS2_Linux.md)

[Try SROS2 on MacOS](SROS2_MacOS.md)

[Try SROS2 on Windows](SROS2_Windows.md)
