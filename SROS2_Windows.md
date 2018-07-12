# Testing on Windows 10 (Tested with Fast-RTPS only)

## Installation

### Install ROS2

#### Install ROS2 from binaries

Please follow [these instructions](https://github.com/ros2/ros2/wiki/Windows-Install-Binary)

#### Install ROS2 from source

Please follow [these instructions](https://github.com/ros2/ros2/wiki/Windows-Development-Setup) and stop at the beginning of "Build the code" section

To build the ROS2 code with security extensions, call:
```bat
colcon build --cmake-args -DSECURITY=ON
```

### Install OpenSSL

If you don't have OpenSSL installed, please see follow [these instructions](https://github.com/ros2/ros2/wiki/Windows-Install-Binary#install-openssl)

### Additional configuration for RTI Connext

To use DDS-Security with Connext you will need to procure an RTI Licence and install the security plugin.
See [this page](https://github.com/ros2/ros2/wiki/Install-Connext-Security-Plugins) for details on installing the security plugins.


## Preparing the environment for the demo

### Create a folder for the files required by this demo

We will now create a folder to store all the files necessary for this demo:

```bat
md C:\dev\ros2\sros2_demo
```

### Generating a keystore, keys and certificates

#### Generate a keystore

```bat
cd sros2_demo
ros2 security create_keystore demo_keys
```

#### Generate keys and certificates for the talker and listener nodes

```bat
ros2 security create_key demo_keys talker
ros2 security create_key demo_keys listener
```

If `unable to write 'random state'` appears then set the environment variable `RANDFILE`.
```bat
set RANDFILE=C:\dev\ros2\sros2_demo\.rnd
```

Then re-run the commands above.

### Define the SROS2 environment variables
Prepare your environment by setting three following environment variables as follows

```bat
set ROS_SECURITY_ROOT_DIRECTORY=%cd%/demo_keys
set ROS_SECURITY_ENABLE=true
set ROS_SECURITY_STRATEGY=Enforce
```


## Run the demo

ROS2 allows you to [change DDS implementation at runtime](https://github.com/ros2/ros2/wiki/Working-with-multiple-RMW-implementations).
This demo can be run with fastrtps by setting:
```bat
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
And with Connext by setting:
```bat
set RMW_IMPLEMENTATION=rmw_connext_cpp
```

Note that secure communication between vendors is not supported.

### Authentication and Encryption

Open a new terminal:

```bat
call <path_to_ros2_install>/setup.bat
set ROS_SECURITY_ROOT_DIRECTORY=%cd%/demo_keys
set ROS_SECURITY_ENABLE=true
set ROS_SECURITY_STRATEGY=Enforce
ros2 run demo_nodes_py talker
```

Open another terminal:

```bat
call <path_to_ros2_install>/setup.bat
set ROS_SECURITY_ROOT_DIRECTORY=%cd%/demo_keys
set ROS_SECURITY_ENABLE=true
set ROS_SECURITY_STRATEGY=Enforce
ros2 run demo_nodes_py listener
```

These nodes will be communicating using authentication and encryption!
If you look at the packet contents on e.g. Wireshark, the messages will be encrypted.

Note: You can switch between the C++ (demo_nodes_cpp) and Python (demo_nodes_py) packages arbitrarily.
 
These nodes are able to communicate because we have created the appropriate keys and certificates for them.
However, other nodes will not be able to communicate, e.g. the following invocation will fail to start a node with a name that is not associated with valid keys/certificates:

```bat
REM This will fail because the node name does not have valid keys/certificates
ros2 run demo_nodes_cpp talker __node:=not_talker
```

### Access Control

The previous demo used authentication and encryption, but not access control, which means that any authenticated node would be able to publish and subscribe to any data stream (aka topic).
To increase the level of security in the system, you can define strict limits, known as access control, which restrict what each node is able to do.
For example, one node would be able to publish to a particular topic, and another node might be able to subscribe to that topic.
To do this, we will use the sample policy file provided in `examples/sample_policy.yaml`.

First, we will copy this sample policy file into our keystore:

```bat
curl -k https://raw.githubusercontent.com/ros2/sros2/master/examples/sample_policy.yaml -o .\demo_keys\policies.yaml
```

And now we will use it to generate the XML permission files expected by the middleware:

```bat
ros2 security create_permission demo_keys talker demo_keys/policies.yaml
ros2 security create_permission demo_keys listener demo_keys/policies.yaml
```

These permission files will be stricter than the ones that were used in the previous demo: the nodes will only be allowed to publish or subscribe to the `chatter` topic (and some other topics used for parameters).

In one terminal (after preparing the terminal as previously described), run the `talker` demo program:

```bat
ros2 run demo_nodes_cpp talker
```

In another terminal (after preparing the terminal as previously described), we will do the same thing with the `listener` program:

```bat
ros2 run demo_nodes_py listener
```

At this point, your `talker` and `listener` nodes should be communicating securely, using explicit access control lists!
Hooray!

The nodes will not be able to publish/subscribe to topics not listed in the policy.
For example, the following attempt for the `listener` node to subscribe to a topic other than `chatter` will fail:

```bat
REM This will fail because the node is not permitted to subscribe to topics other than chatter.
ros2 run demo_nodes_py listener chatter:=not_chatter
```
