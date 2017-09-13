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

Note: this will be needed for every terminal you use for testing, defining these environment variables globally will be more convenient.


## Run the demo

Open a new terminal:


```
call <path_to_ros2_install>/setup.bat
ros2 run demo_nodes_py talker
```

Open another terminal:

```
call <path_to_ros2_install>/setup.bat
ros2 run demo_nodes_py listener
```
