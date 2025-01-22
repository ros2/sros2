# Try SROS2 in Linux

## Installation

### Install from debian packages

First install ROS 2 from binaries following [these instructions](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html).

Setup your environment following [these instructions](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html#environment-setup).

In the rest of these instruction we assume that every terminal setup the environment as instructed above.


### Install from source

You will need to have openssl installed on your machine:

```bash
sudo apt update && sudo apt install libssl-dev
```

First install ROS 2 from source following [these instructions](https://docs.ros.org/en/rolling/Installation/Ubuntu-Development-Setup.html).

Note: Fast-RTPS requires an additional CMake flag to build the security plugins so the colcon invocation needs to be modified to pass:
```bash
colcon build --symlink-install --cmake-args -DSECURITY=ON --packages-select fastrtps rmw_fastrtps_cpp rmw_fastrtps_dynamic_cpp rmw_fastrtps_shared_cpp
```

### Additional configuration for RTI Connext

Prerequisite: to use DDS-Security with Connext you will need to procure an RTI Licence and install the security plugins (note that you also need to install RTI's version of openssl as 5.3.1 doesn't support openssl 1.1.0 provided in Ubuntu Bionic).
See [this page](https://docs.ros.org/en/rolling/Installation/DDS-Implementations/Install-Connext-Security-Plugins.html) for details on installing the security plugins.

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
ros2 security create_keystore demo_keystore
```

#### Generate keys and certificates for the talker and listener nodes

```bash
ros2 security create_enclave demo_keystore /talker_listener/talker
ros2 security create_enclave demo_keystore /talker_listener/listener
```

### Define the SROS2 environment variables

```bash
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

These variables need to be defined in each terminal used for the demo. For convenience you can add it to your `~/.bashrc`.

### Run the demo

ROS 2 allows you to [change DDS implementation at runtime](https://docs.ros.org/en/rolling/Guides/Working-with-multiple-RMW-implementations.html).
This demo can be run with FastDDS / CycloneDDS / ConnextDDS by setting the `RMW_IMPLEMENTATION` variable, e.g.:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp  # or
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # or
export RMW_IMPLEMENTATION=rmw_connextdds
```

Note that secure communication between vendors is not supported.

Run the `talker` demo program:

```bash
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
```

In another terminal (after preparing the terminal as previously described), we will do the same thing with the `listener` program.

```bash
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener
```

These nodes will be communicating using authentication and encryption!
If you look at the packet contents on e.g. Wireshark, the messages will be encrypted.

Note: You can switch between the C++ (demo_nodes_cpp) and Python (demo_nodes_py) packages arbitrarily.

These nodes are able to communicate because we have created the appropriate keys and certificates for them.

To be able to use the ros2 CLI tools to interact with your secured system, you need to provide it with an override enclave:
```bash
export ROS_SECURITY_ENCLAVE_OVERRIDE=/talker_listener/listener
```

Then use the CLI with `--no-daemon` and `--spin-time`:

> [!NOTE]
> Avoid using `ros2 daemon` since it may not have security enclaves, and enough time duration should be given for the discovery in secured network.

```bash
ros2 node list --no-daemon --spin-time 4
```
```
/talker
```
```bash
ros2 topic list --no-daemon --spin-time 4
```
```
/chatter
/parameter_events
/rosout
```
```bash
ros2 topic echo /chatter --spin-time 4
```
```
[INFO] [1714897092.882384995] [rcl]: Found security directory: /root/sros2_demo/demo_keystore/enclaves/talker_listener/listener
data: 'Hello World: 257'
---
data: 'Hello World: 258'
---

```

### Run the demo on different machines

The previous demo was using SROS 2 on the same box over localhost.
That's great, but it's more exciting when multiple machines are involved, since the benefits of authentication and encryption are more obvious.

Let's say that the machine with the keystore created in the previous demo has a hostname `feather2`, and that we want to also use another machine with hostname `oldschool` for our multi-machine talker/listener demo.
First, we need to run the installation and compilation steps described previously on the `oldschool` machine.
Then, we need to copy some keys to `oldschool` to allow SROS 2 to authenticate and encrypt the transmissions.
Since the keys are just text files, we can use `scp` to copy them.
First, we'll create an empty keystore on `oldschool` with a `talker_listener` enclave directory:

```
ssh oldschool.local "mkdir -p ~/sros2_demo/demo_keystore/enclaves/talker_listener"
```

Now, we'll copy the keys/certificates for the "talker" program from `feather2` to `oldschool`:

```
cd ~/sros2_demo/demo_keystore/enclaves/talker_listener
scp -r talker oldschool.local:~/sros2_demo/demo_keystore/enclaves/talker_listener
```

That will be very quick, since it's just copying some very small text files.
Now, we're ready to run a multi-machine talker/listener demo!

Once the environment is setup we can run on oldschool:

```bash
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
```


and on feather2

```bash
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener
```


### Access Control

The previous demo used authentication and encryption, but not access control, which means that any authenticated node would be able to publish and subscribe to any data stream (aka topic).
To increase the level of security in the system, you can define strict limits, known as access control, which restrict what each node is able to do.
For example, one node would be able to publish to a particular topic, and another node might be able to subscribe to that topic.
To do this, we will use the sample policy file provided in `examples/sample_policy.xml`.

First, we will copy this sample policy file into our keystore:

```bash
sudo apt update && sudo apt install git
cd ~/sros2_demo
git clone https://github.com/ros2/sros2.git /tmp/sros2
```

And now we will use it to generate the XML permission files expected by the middleware:

```bash
ros2 security create_permission demo_keystore /talker_listener/talker /tmp/sros2/sros2/test/policies/sample.policy.xml
ros2 security create_permission demo_keystore /talker_listener/listener /tmp/sros2/sros2/test/policies/sample.policy.xml
```

These permission files will be stricter than the ones that were used in the previous demo: the nodes will only be allowed to publish or subscribe to the `chatter` topic (and some other topics used for parameters).

In one terminal (after preparing the terminal as previously described), run the `talker` demo program:

```
ros2 run demo_nodes_cpp talker --ros-args -e /talker_listener/talker
```

In another terminal (after preparing the terminal as previously described), we will do the same thing with the `listener` program:

```
ros2 run demo_nodes_py listener --ros-args -e /talker_listener/listener
```

At this point, your `talker` and `listener` nodes should be communicating securely, using explicit access control lists!
Hooray!

The nodes will not be able to publish/subscribe to topics not listed in the policy.
For example, the following attempt for the `listener` node to subscribe to a topic other than `chatter` will fail:

```bash
# This will fail because the node is not permitted to subscribe to topics other than chatter.
ros2 run demo_nodes_py listener --ros-args -r chatter:=not_chatter -e /talker_listener/listener
```

### Certificate Revocation Lists

In some circumstances, it may be necessary to revoke certificates before they have expired.
This is accomplished with Certificate Revocation Lists (CRLs), and is supported by SROS2 security enclaves.

Following on from the previous demo, let's assume that we want to revoke the certificate for the listener.
To do this, we first need to generate a crl.pem file in the security enclave:

```bash
cd ~/sros2_demo/demo_keystore
cat > crl_openssl.conf << EOF
# OpenSSL configuration for CRL generation

[ ca ]
default_ca = CA_default

[ CA_default ]
database = index.txt
crlnumber = crlnumber

default_days = 365  # how long to certify for
default_crl_days = 30  # how long before next CRL
default_md = default  # use public key default MD
preserve = no  # keep passed DN ordering

[ crl_ext ]
# CRL extensions.
# Only issuerAltName and authorityKeyIdentifier make any sense in a CRL.
# issuerAltName=issuer:copy
authorityKeyIdentifier = keyid:always,issuer:always
EOF
echo 00 > crlnumber
touch index.txt
openssl ca -revoke enclaves/talker_listener/listener/cert.pem -keyfile private/identity_ca.key.pem -cert public/identity_ca.cert.pem -config crl_openssl.conf
openssl ca -gencrl -keyfile private/identity_ca.key.pem -cert public/identity_ca.cert.pem -out public/crl.pem -config crl_openssl.conf
```

Now we need to link it into the talker enclave (so it will reject connection attempts by the listener):

```bash
ln -s ../../../public/crl.pem enclaves/talker_listener/talker
```

Now we can run the talker demo as above (after preparing the terminal as previously described):
```bash
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker
```

You'll notice that this is operating just like before.

In another terminal (after preparing the terminal as previously described), we will do the same thing with the `listener` program.

```bash
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener
```

Here you'll notice that the listener is not getting any data.
That's because the talker is explicitly rejecting the listener revoked certificate of the listener, so no communication is possible.
