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

Note: Fast-RTPS requires an additional CMake flag to build the security plugins so the colcon invocation needs to be modified to pass:
```bash
colcon build --symlink-install --cmake-args -DSECURITY=ON
```

### Additional configuration for RTI Connext

Prerequisite: to use DDS-Security with Connext you will need to procure an RTI Licence and install the security plugins (note that you also need to install RTI's version of openssl as 5.3.1 doesnt support openssl 1.1.0 provided in Ubuntu Bionic).
See [this page](https://github.com/ros2/ros2/wiki/Install-Connext-Security-Plugins) for details on installing the security plugins.

Warning: this tutorial is for ROS Bouncy and Connext 5.3.1.
If you use ROS Ardent or Connext 5.3.0 please refer to the [tutorial from ROS Ardent](https://github.com/ros2/sros2/blob/ardent/SROS2_Linux.md)

The RTI Connext installer allows you to choose where it lands in the filesystem.
These instructions assume that you have prefixed the RTI paths with `$HOME/rti` so that the latest version will land in `$HOME/rti/rti_connext_dds-5.3.1`.
Note that the installer is a multi-partprocess.
Fist you must install the "host" package, and then from its launcher you can install the additional "target" packages and the `secure_dds` package.
Additional (and better) help is provided in the RTI documentation.

```bash
source ~/rti/rti_connext_dds-5.3.1/resource/scripts/rtisetenv_x64Linux3gcc5.4.0.bash'
```

It's often handy to create an alias for that super-long shell incantation to source the RTI script, for example, by adding something like this to your `~/.bashrc` file (altering the paths and alias names as needed/wanted):

```
alias rti='. ~/rti/rti_connext_dds-5.3.1/resource/scripts/rtisetenv_x64Linux3gcc5.4.0.bash'
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

ROS2 allows you to [change DDS implementation at runtime](https://github.com/ros2/ros2/wiki/Working-with-multiple-RMW-implementations).
This demo can be run with fastrtps by setting:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
And with Connext by setting:
```bash
export RMW_IMPLEMENTATION=rmw_connext_cpp
```

Note that secure communication between vendors is not supported.

Run the `talker` demo program:

```bash
ros2 run demo_nodes_cpp talker
```

In another terminal (after preparing the terminal as previously described), we will do the same thing with the `listener` program.

```bash
ros2 run demo_nodes_py listener
```

These nodes will be communicating using authentication and encryption!
If you look at the packet contents on e.g. Wireshark, the messages will be encrypted.

Note: You can switch between the C++ (demo_nodes_cpp) and Python (demo_nodes_py) packages arbitrarily.

These nodes are able to communicate because we have created the appropriate keys and certificates for them.
However, other nodes will not be able to communicate, e.g. the following invocation will fail to start a node with a name that is not associated with valid keys/certificates:

```bash
# This will fail because the node name does not have valid keys/certificates
ros2 run demo_nodes_cpp talker __node:=not_talker
```


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

Once the environment is setup we can run on oldschool:

```bash
ros2 run demo_nodes_cpp talker
```


and on feather2

```bash
ros2 run demo_nodes_py listener
```


### Access Control

The previous demo used authentication and encryption, but not access control, which means that any authenticated node would be able to publish and subscribe to any data stream (aka topic).
To increase the level of security in the system, you can define strict limits, known as access control, which restrict what each node is able to do.
For example, one node would be able to publish to a particular topic, and another node might be able to subscribe to that topic.
To do this, we will use the sample policy file provided in `examples/sample_policy.yaml`.

First, we will copy this sample policy file into our keystore:

```bash
curl -sk https://raw.githubusercontent.com/ros2/sros2/master/examples/sample_policy.yaml -o ./demo_keys/policies.yaml
```

And now we will use it to generate the XML permission files expected by the middleware:

```bash
ros2 security create_permission demo_keys talker demo_keys/policies.yaml
ros2 security create_permission demo_keys listener demo_keys/policies.yaml
```

These permission files will be stricter than the ones that were used in the previous demo: the nodes will only be allowed to publish or subscribe to the `chatter` topic (and some other topics used for parameters).

In one terminal (after preparing the terminal as previously described), run the `talker` demo program:

```
ros2 run demo_nodes_cpp talker
```

In another terminal (after preparing the terminal as previously described), we will do the same thing with the `listener` program:

```
ros2 run demo_nodes_py listener
```

At this point, your `talker` and `listener` nodes should be communicating securely, using explicit access control lists!
Hooray!

The nodes will not be able to publish/subscribe to topics not listed in the policy.
For example, the following attempt for the `listener` node to subscribe to a topic other than `chatter` will fail:

```bash
# This will fail because the node is not permitted to subscribe to topics other than chatter.
ros2 run demo_nodes_py listener chatter:=not_chatter
```
