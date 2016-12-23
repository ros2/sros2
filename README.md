# CONSTRUCTION ZONE

WARNING: all of this is under construction! Things will be highly volatile for
a while.

# Installing the SROS2 prototype

Although we are designing SROS2 to work with any secure middleware, at the
moment we are only testing with RTI Connext Secure. As such, to operate this
demonstration right now (December 2016), you will need a license for RTI
Connext Secure in order to try out this demo.

## Install RTI Connext Secure

The RTI Connext installer allows you to choose where it lands in the
filesystem. These instructions assume that you have prefixed the RTI paths with
`$HOME/rti` so that the latest version (5.2.4 at time of writing) will land in
`$HOME/rti/rti_connext_dds-5.2.4`  Note that the installer is a multi-part
process; fist you must install the "host" package, and then from its launcher
you can install the additional "target" packages and the `secure_dds` package.
Much more (and better) help is provided in the RTI documentation.

## Download the SROS 2 demo source tree

The following instructions will download a version of the ROS 2 source tree
with a few repositories checked out to the `sros2` branch.
```
mkdir -p ~/sros2/src
cd ~/sros2/src
wget https://raw.githubusercontent.com/ros2/sros2/master/sros2.repos
vcs import < ../sros2.repos
```

## Build the SROS 2 demo source tree

Before building the SROS 2 source tree, you need to source the RTI Connext
configuration file, which sets some required environment variables that point
to your RTI installation path. Then, you can start the SROS 2 build:

```
source ~/rti/rti_connext_dds-5.2.4/resource/scripts/rtisetenv_x64Linux3gcc4.8.2.bash
cd ~/sros2
src/ament/ament_tools/scripts/ament.py build -s
```
