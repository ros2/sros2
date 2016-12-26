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

# Demos

## Getting terminals ready

For every terminal window used in these demos, you'll need to first source the
RTI setup script, and then source the setup script from your installed SROS 2
workspace, like this:

```
source ~/rti/rti_connext_dds-5.2.4/resource/scripts/rtisetenv_x64Linux3gcc4.8.2.bash
cd ~/sros2
source install/setup.bash
```

## Secure Talker Listener

In one terminal (after preparing the environment as previously described), we
will create a keystore in `~/sros2/demo_keys` :
```
sros2 create_keystore demo_keys
sros2 create_key talker
sros2 create_key listener
```

Then, in one terminal (after preparing the terminal as previously described),
we can set the `ROS_SECURE_ROOT` to our keystore path, and then run the
`talker` demo program:
```
ROS_SECURE_ROOT=~/sros2/demo_keys talker
```

In another terminal (after preparing the terminal as previously described), we
will do the same thing with the `listener` program:
```
ROS_SECURE_ROOT=~/sros2/demo_keys listener
```

At thsi point, your `talker` and `listener` nodes should be communicating
securely! Hooray!

# Tips and Tricks

## RTI Connext Secure environment variables

It's often handy to create an alias for that super-long shell incantation to
source the RTI script, for example, by adding something like this to your
`~/.bashrc` file (altering the paths and alias names as needed/wanted):

```
alias rti='. ~/rti/rti_connext_dds-5.2.4/resource/scripts/rtisetenv_x64Linux3gcc4.8.2.bash'
```

