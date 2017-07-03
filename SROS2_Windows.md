# Testing on Windows 10 (Tested with Fast-RTPS only)

## Install OpenSSL

Download an OpenSLL installer from . Scroll to the bottom of [this page](https://slproweb.com/products/Win32OpenSSL.html) and download *Win64 OpenSSL v1.0.2*. Don't download the Win32 or Light versions.

Run the installer with default parameters. Then, define environment variables (the following commands assume you used the default installation directory):

- `set OPENSSL_CONF=C:\OpenSSL-Win64\bin\openssl.cfg`
- Append `C:\OpenSSL-Win64\bin\` to your PATH

Note: you will need this in all terminals so setting these environment variables globally will be more convenient.

## Install ROS2 from binaries

Please follow [these instructions](https://github.com/ros2/ros2/wiki/Windows-Install-Binary)

## Install ROS2 from source

Please follow [these instructions](https://github.com/ros2/ros2/wiki/Windows-Development-Setup) and stop at the beginning of "Build the code" section

To build the ROS2 code with security extensions, call:
```bat
python src\ament\ament_tools\scripts\ament.py build --build-tests --cmake-args -DSECURITY=ON --
```


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

Note: this will be needed for every terminal you use for testing, defining these environment variables globally will be more convenient.

Open a new terminal:


```
cd C:\dev\ros2
call setup.bat
ros2 run demo_nodes_py talker
```

Open another terminal:

```
cd C:\dev\ros2
call setup.bat
ros2 run demo_nodes_py listener
```
