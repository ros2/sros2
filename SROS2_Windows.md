# Testing on Windows 10 (Tested with Fast-RTPS only)

## Installation

### Install ROS2

#### Install ROS2 from binaries

Please follow [these instructions](https://github.com/ros2/ros2/wiki/Windows-Install-Binary)

#### Install ROS2 from source

Please follow [these instructions](https://github.com/ros2/ros2/wiki/Windows-Development-Setup) and stop at the beginning of "Build the code" section

To build the ROS2 code with security extensions, call:
```bat
python src\ament\ament_tools\scripts\ament.py build --build-tests --cmake-args -DSECURITY=ON --
```

### Install OpenSSL

If you don't have OpenSSL installed, please see follow [these instructions](https://github.com/ros2/ros2/wiki/Windows-Install-Binary#install-openssl)

## Preparing the environment for the demo

### Create a folder for the files required by this demo

We will now create a folder to store all the files necessary for this demo:

```bash
md C:\dev\ros2\sros2_demo
```

### Generating a keystore, keys and certificates

#### Generate a keystore

```bash
cd sros2_demo
ros2 security create_keystore demo_keys
```

#### Generate keys and certificates for the talker and listener nodes

```bash
ros2 security create_key demo_keys talker
ros2 security create_key demo_keys listener
```

### Define the SROS2 environment variables
Prepare your environment by setting three following environment variables as follows

- `set ROS_SECURITY_ROOT_DIRECTORY=%cd%/demo_keys`
- `set ROS_SECURITY_ENABLE=true`
- `set ROS_SECURITY_STRATEGY=Enforce`


## Run the demo

ROS2 allows you to [change DDS implementation at runtime](https://github.com/ros2/ros2/wiki/Working-with-multiple-RMW-implementations).
This demo can be run with fastrtps by setting:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
And with Connext by setting:
```bash
export RMW_IMPLEMENTATION=rmw_connext_cpp
```

### Authentication and Encryption

Open a new terminal:

```
call <path_to_ros2_install>/setup.bat
set ROS_SECURITY_ROOT_DIRECTORY=%cd%/demo_keys`
set ROS_SECURITY_ENABLE=true`
set ROS_SECURITY_STRATEGY=Enforce`
ros2 run demo_nodes_py talker
```

Open another terminal:

```
call <path_to_ros2_install>/setup.bat
set ROS_SECURITY_ROOT_DIRECTORY=%cd%/demo_keys`
set ROS_SECURITY_ENABLE=true`
set ROS_SECURITY_STRATEGY=Enforce`
ros2 run demo_nodes_py listener
```

For comparison if you open another terminal and only run:

```
call <path_to_ros2_install>/setup.bat
ros2 run demo_nodes_py listener
```

You will see that it cannot connect and receive the messages.

### Access Control (RTI Connext only, from source only)

The previous demo used authentication and encryption, but not access control, which means that any authenticated node would be able to publish and subscribe to any data stream (aka topic).
To increase the level of security in the system, you can define strict limits, known as access control, which restrict what each node is able to do.
For example, one node would be able to publish to a particular topic, and another node might be able to subscribe to that topic.
To do this, we will use the sample policy file provided in `examples/sample_policy.yaml`.

First, we will copy this sample policy file into our keystore:

```bash
curl -sk https://raw.githubusercontent.com/ros2/sros2/ardent/examples/sample_policy.yaml -o .\demo_keys\policies.yaml
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
