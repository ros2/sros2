# Context

This package provides the tools and instructions to use ROS2 on top of DDS-Security.
The security feature is tested across platforms (Linux, macOS, and Windows) as well as across different languages (C++ and Python).

# Installing the SROS2 prototype

Although we are designing SROS2 to work with any secure middleware, at the moment we are only testing with RTI Connext Secure and eProsima's Fast-RTPS.
This package provides a docker container for Fast-RTPS testing.
If you want to run the demo using RTI Connext Secure you will need a license for it and you will need to install it.

## Testing in a docker container (Fast-RTPS only)

To setup the environment, first clone this repository:

```
mkdir ~/sros2_demo && cd ~/sros2_demo
git clone https://github.com/ros2/sros2.git
```

And then build the docker container:

```
cd ~/sros2_demo/sros2/sros2_docker/docker
./build_sros2.sh
```

This may take a few minutes.

Finally enter the container:

```
./run_sros2_container.sh
```

Hooray we are now ready to start the demo.

We will start by creating a set of keys for our DDS participant:

```
cd /root/sros2_ws
ros2 security create_keystore demo_keys
ros2 security create_key demo_keys talker
ros2 security create_key demo_keys listener
```

And now start testing!

In the terminal inside docker, run:

```
talker_py
```

This will start a python executable that publishes messages periodically.

Now let's open another terminal into the container by opening a new terminal, then running:

```
docker exec -ti <CONTAINER_NAME> /bin/bash
listener
```

Hooray our nodes are authenticated and talking encrypted-talk!

## Testing on Linux

### Install some dependencies

These instructions assume Ubuntu 16.04.
You can likely make this work on many other Linux distros, but you'd have to chase down the equivalent dependency packages.
This is just an abbreviated version of the dependencies described in the main ROS 2 installation page:

https://github.com/ros2/ros2/wiki/Linux-Development-Setup

First, we'll add the OSRF apt repositories to your system, so that we can pull down some of the ROS tools which will be used to build SROS 2:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
sudo apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
```

Now we'll install a bunch of apt packages:

```
sudo apt-get update
sudo apt-get install git wget build-essential cmake cppcheck libopencv-dev libpoco-dev libpocofoundation9v5 libpocofoundation9v5-dbg python-empy python3-dev python3-empy python3-nose python3-pip python3-setuptools python3-vcstool libboost-chrono-dev libboost-date-time-dev libboost-program-options-dev libboost-regex-dev libboost-system-dev libboost-thread-dev libssl-dev openssl
```

## Optional: install RTI Connext Secure

The RTI Connext installer allows you to choose where it lands in the filesystem.
These instructions assume that you have prefixed the RTI paths with `$HOME/rti` so that the latest version (5.2.4 at time of writing) will land in `$HOME/rti/rti_connext_dds-5.2.4`.
Note that the installer is a multi-partprocess.
Fist you must install the "host" package, and then from its launcher you can install the additional "target" packages and the `secure_dds` package.
Additional (and better) help is provided in the RTI documentation.


## Download the SROS 2 demo source tree

The following instructions will download a version of the ROS 2 source tree with a few repositories checked out to the `sros2` branch.

```
mkdir -p ~/sros2/src
cd ~/sros2
wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
vcs import src < ros2.repos
```

## Prepare your environment (RTI Connext only)

Before building the SROS 2 source tree, you need to source the RTI Connext configuration file, which sets some required environment variables that point to your RTI installation path:

```
source ~/rti/rti_connext_dds-5.2.4/resource/scripts/rtisetenv_x64Linux3gcc4.8.2.bash
```

Note: you will have to repeat this for every terminal you open

## Build the SROS 2 demo source tree

```bash
cd ~/sros2
src/ament/ament_tools/scripts/ament.py build -s --cmake-args -DSECURITY=ON --
```

# Demos

## Getting terminals ready

For every terminal window used in these demos, you'll need to source the setup script from your installed SROS 2 workspace, like this:

```bash
cd ~/sros2
source install/setup.bash
```

## Secure Talker Listener

### Create security files (keys and certificate) and define environment variables for ROS2 security

In one terminal (after preparing the environment as previously described), we will create a keystore in `~/sros2/demo_keys`:

```bash
ros2 security create_keystore demo_keys
```

Generate certificates and keys for our `talker` and `listener` nodes:

```bash
ros2 security create_key demo_keys talker
ros2 security create_key demo_keys listener
```

Then, in one terminal (after preparing the terminal as previously described), we can set the environment variables used for ROS2 Security:

```bash
export ROS_SECURITY_ROOT_DIRECTORY=~/sros2/demo_keys
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

Note: for convenience you may add these lines to your `~/.bashrc`.

Run the `talker` demo program for either Fast-RTPS:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp talker
```

Of RTI Connext:

```bash
RMW_IMPLEMENTATION=rmw_connext_cpp talker
```

In another terminal (after preparing the terminal as previously described), we will do the same thing with the `listener` program.
Again, for either Fast-RTPS:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp listener
```

Or RTI Connext:

```bash
RMW_IMPLEMENTATION=rmw_connext_cpp listener
```

At this point, your `talker` and `listener` nodes should be communicating securely, using authentication and encryption!
Hooray!

Note: You could also run the Python version of these nodes: `talker_py` and `listener_py`

## Access Control (RTI Connext only)

The previous demo used authentication and encryption, but not access control, which means that any authenticated node would be able to publish and subscribe to any data stream (aka topic).
To increase the level of security in the system, you can define strict limits, known as access control, which restrict what each node is able to do.
For example, one node would be able to publish to a particular topic, and another node might be able to subscribe to that topic.
To do this, we will use the sample policy file provided in `examples/sample_policy.yaml`.

First, we will copy this sample policy file into our keystore:

```bash
cp ~/sros2/src/ros2/sros2/examples/sample_policy.yaml ./demo_keys/
```

And now we will use it to generate the XML permission files expected by the middleware:

```bash
ros2 security create_permission demo_keys talker demo_keys/policies.yaml
ros2 security create_permission demo_keys listener demo_keys/policies.yaml
```

Then, in one terminal (after preparing the terminal as previously described), run the `talker` demo program:

```
RMW_IMPLEMENTATION=rmw_connext_cpp talker
```

In another terminal (after preparing the terminal as previously described), we will do the same thing with the `listener` program:

```
RMW_IMPLEMENTATION=rmw_connext_cpp listener
```

At this point, your `talker` and `listener` nodes should be communicating securely, using explicit access control lists!
Hooray!

## Two different machines

The previous demo was using SROS 2 on the same box over localhost.
That'sgreat, but it's more exciting when multiple machines are involved, since the benefits of authentication and encryption are more obvious.

Let's say that the machine with the keystore created in the previous demo has a hostname `feather2`, and that we want to also use another machine with hostname `oldschool` for our multi-machine talker/listener demo.
First, we need to run the installation and compilation steps described previously on the `oldschool` machine.
Then, we need to copy some keys to `oldschool` to allow SROS 2 to authenticate and encrypt the transmissions.
Since the keys are just text files, we can use `scp` to copy them.
First, we'll create an empty keystore on `oldschool`, which is just an empty directory:

```
ssh oldschool.local
mkdir -p ~/sros2/demo_keys
exit
```

Now, we'll copy the keys/certificates for the "talker" program from `feather2` to `oldschool`:

```
cd ~/sros2/demo_keys
scp -r talker USERNAME@oldschool.local:~/sros2/demo_keys
```

That will be very quick, since it's just copying some very small text files.
Now, we're ready to run a multi-machine talker/listener demo!

First, on the machine running `talker`, we need to source the RTI variables, the SROS 2 installation tree, and then we can run `talker` in secure mode by setting the `ROS_SECURITY_ROOT_DIRECTORY` environment variable to the local keystore:

```
source ~/rti/rti_connext_dds-5.2.4/resource/scripts/rtisetenv_x64Linux3gcc4.8.2.bash'
source ~/sros2/install/setup.bash
RMW_IMPLEMENTATION=rmw_connext_cpp talker
```

# Tips and Tricks

## RTI Connext Secure environment variables

It's often handy to create an alias for that super-long shell incantation to source the RTI script, for example, by adding something like this to your `~/.bashrc` file (altering the paths and alias names as needed/wanted):

```
alias rti='. ~/rti/rti_connext_dds-5.2.4/resource/scripts/rtisetenv_x64Linux3gcc4.8.2.bash'
```

# Testing on Windows 10 (Tested with Fast-RTPS only)

## Install dependencies

These instructions assume Windows 10 64-bit.
Please follow the instrucions on https://github.com/ros2/ros2/wiki/Windows-Development-Setup and stop at the "Getting the source code" section

### Install OpenSSL

Download an OpenSLL installer from . Scroll to the bottom of [this page](https://slproweb.com/products/Win32OpenSSL.html) and download *Win64 OpenSSL v1.0.2*. Don't download the Win32 or Light versions.

Run the installer with default parameters. Then, define environment variables (the following commands assume you used the default installation directory):

- `set OPENSSL_CONF=C:\OpenSSL-Win64\bin\openssl.cfg`
- Append `C:\OpenSSL-Win64\bin\` to your PATH

## Creating keys and certificates

```
ros2 security create_keystore demo_keys
ros2 security create_key demo_keys talker
ros2 security create_key demo_keys listener
```

## Testing

Prepare your environment by setting three following environment variables as follows

- `set ROS_SECURITY_ROOT_DIRECTORY=%cd%/demo_keys`
- `set ROS_SECURITY_ENABLE=true`
- `set ROS_SECURITY_STRATEGY=Enforce`

Open a new terminal:

```
cd C:\dev\sros2
call setup.bat
ros2 run demo_nodes_py talker
```

Open another terminal:

```
cd C:\dev\sros2
call setup.bat
ros2 run demo_nodes_py listener
```
