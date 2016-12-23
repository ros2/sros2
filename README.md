# Installing the SROS2 prototype

The following instructions will build a version of the ROS 2 source tree with a
few repositories checked out to the `sros2` branch. At time of writing, this
prototype will only work with RTI Connext Secure, as exposed through the
`rmw_connext_cpp` ROS middleware abstraction layer.

```
mkdir -p ~/sros2/src
cd ~/sros2
wget https://raw.githubusercontent.com/ros2/sros2/master/sros2.repos
```
