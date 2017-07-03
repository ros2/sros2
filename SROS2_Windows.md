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
