# Security Helper
Add node authentication, cryptography, and access control security keys using a cmake macro.
The macro will generate the secure root directory if it does not exists, then create authentication and cryptography keys in the secure root directory.

In package.xml add:  
`<depend>sros2_cmake</depend>`  
In CMakeLists add:  
`find_package(sros2_cmake REQUIRED)`  
`ros2_secure_node(NODES <node_name>)`  

Macro definition:  
```
    # ros2_secure_node(NODES <node_1> <node_2>...<node_n>)

    # NODES (macro multi-arg) takes the node names for which keys will be generated
    # SECURITY (cmake arg) if not define or OFF, will not generate key/keystores
    # ROS_SECURITY_ROOT_DIRECTORY (env variable) the location of the keystore
    # POLICY_FILE (cmake arg) if defined, will compile policies by node name into the access private certificates (e.g POLICY_FILE=/etc/policies/<policy.xml>, Generate: <node_name> /etc/policies/<policy.xml>) **if defined, all nodes must have a policy defined for them**
```
