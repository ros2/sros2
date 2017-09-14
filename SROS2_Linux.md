# Try SROS2 in Linux

## Installation

### Install from debian packages

First install ROS2 from binaries following [these instructions](https://github.com/ros2/ros2/wiki/Linux-Install-Debians)

Setup your environment following [these instructions](https://github.com/ros2/ros2/wiki/Linux-Install-Debians#environment-setup)

In the rest of these instruction we assume that every terminal setup the environment as instructed above.


### Install from source

You will need to have openssl installed on your machine:

```bash
sudo apt update && sudo apt install libssl-dev
```

First install ROS2 from source following [these instructions](https://github.com/ros2/ros2/wiki/Linux-Development-Setup)

Note: Fast-RTPS requires an additional CMake flag to build the security plugins so the ament invocation needs to be modified to pass:
```bash
src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install --cmake-args -DSECURITY=ON --
```

### Additional configuration for RTI Connext

Prerequisite: to use DDS-Scurity with connext you will need to procure an RTI Licence and install the security plugin.

Warning: this tutorial is for Connext 5.3.0. If you use Connext 5.2.4 please refer to the [tutorial from ROS 2 Beta 3](https://github.com/ros2/sros2/blob/release-beta3/SROS2_Linux.md)

The RTI Connext installer allows you to choose where it lands in the filesystem.
These instructions assume that you have prefixed the RTI paths with `$HOME/rti` so that the latest version (5.3.0 at time of writing) will land in `$HOME/rti/rti_connext_dds-5.3.0`.
Note that the installer is a multi-partprocess.
Fist you must install the "host" package, and then from its launcher you can install the additional "target" packages and the `secure_dds` package.
Additional (and better) help is provided in the RTI documentation.

```bash
source ~/rti/rti_connext_dds-5.3.0/resource/scripts/rtisetenv_x64Linux3gcc5.4.0.bash'
```

It's often handy to create an alias for that super-long shell incantation to source the RTI script, for example, by adding something like this to your `~/.bashrc` file (altering the paths and alias names as needed/wanted):

```
alias rti='. ~/rti/rti_connext_dds-5.3.0/resource/scripts/rtisetenv_x64Linux3gcc5.4.0.bash'
```

## Run the demo

### Create a folder for the files required by this demo

We will now create a folder to store all the files necessary for this demo:

```bash
mkdir ~/sros2_demo
```

### Generating a keystore, keys and certificates

#### Generate a keystore

```bash
cd ~/sros2_demo
ros2 security create_keystore demo_keys
```

#### Generate keys and certificates for the talker and listener nodes

```bash
ros2 security create_key demo_keys talker
ros2 security create_key demo_keys listener
```

### Define the SROS2 environment variables

```bash
export ROS_SECURITY_ROOT_DIRECTORY=~/sros2_demo/demo_keys
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

These variables need to be defined in each terminal used for the demo. For convenience you can add it to your `~/.bashrc`.

### Run the demo

Run the `talker` demo program for either Fast-RTPS:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run demo_nodes_cpp talker
```

Of RTI Connext:

```bash
RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run demo_nodes_cpp talker
```

In another terminal (after preparing the terminal as previously described), we will do the same thing with the `listener` program.
Again, for either Fast-RTPS:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run demo_nodes_py listener
```

Or RTI Connext:

```bash
RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run demo_nodes_py listener
```

Note: You can switch between the C++ and Python packages arbitrarily.


### Run the demo on different machines

The previous demo was using SROS 2 on the same box over localhost.
That's great, but it's more exciting when multiple machines are involved, since the benefits of authentication and encryption are more obvious.

Let's say that the machine with the keystore created in the previous demo has a hostname `feather2`, and that we want to also use another machine with hostname `oldschool` for our multi-machine talker/listener demo.
First, we need to run the installation and compilation steps described previously on the `oldschool` machine.
Then, we need to copy some keys to `oldschool` to allow SROS 2 to authenticate and encrypt the transmissions.
Since the keys are just text files, we can use `scp` to copy them.
First, we'll create an empty keystore on `oldschool`, which is just an empty directory:

```
ssh oldschool.local
mkdir -p ~/sros2_demo/demo_keys
exit
```

Now, we'll copy the keys/certificates for the "talker" program from `feather2` to `oldschool`:

```
cd ~/sros2_demo/demo_keys
scp -r talker USERNAME@oldschool.local:~/sros2_demo/demo_keys
```

That will be very quick, since it's just copying some very small text files.
Now, we're ready to run a multi-machine talker/listener demo!

Once the envrionment setup we can run on oldschool, for either Fast-RTPS:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run demo_nodes_cpp talker
```

Of RTI Connext:

```bash
RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run demo_nodes_cpp talker
```

and on feather2
Again, for either Fast-RTPS:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run demo_nodes_py listener
```

Or RTI Connext:

```bash
RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run demo_nodes_py listener
```


### Access Control (RTI Connext only, from source only)

The previous demo used authentication and encryption, but not access control, which means that any authenticated node would be able to publish and subscribe to any data stream (aka topic).
To increase the level of security in the system, you can define strict limits, known as access control, which restrict what each node is able to do.
For example, one node would be able to publish to a particular topic, and another node might be able to subscribe to that topic.
To do this, we will use the sample policy file provided in `examples/sample_policy.yaml`.

First, we will copy this sample policy file into our keystore:

```bash
cp ~/ros2_ws/src/ros2/sros2/examples/sample_policy.yaml ./demo_keys/policies.yaml
```

And now we will use it to generate the XML permission files expected by the middleware:

```bash
ros2 security create_permission demo_keys talker demo_keys/policies.yaml
ros2 security create_permission demo_keys listener demo_keys/policies.yaml
```

Then, in one terminal (after preparing the terminal as previously described), run the `talker` demo program:

```
RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run demo_nodes_cpp talker
```

In another terminal (after preparing the terminal as previously described), we will do the same thing with the `listener` program:

```
RMW_IMPLEMENTATION=rmw_connext_cpp ros2 run demo_nodes_py listener
```

At this point, your `talker` and `listener` nodes should be communicating securely, using explicit access control lists!
Hooray!
