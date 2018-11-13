# Security Helper
Add node authentication, cryptography, and access control security keys using a cmake macro.

In package.xml add:  
`<depend>ros2_security_helpers</depend>`
In CMakeLists add:  
`find_package(ros2_security_helpers REQUIRED)`
Then use the macro:  
    # ros2_secure_node(NODES <node_1> <node_2>...<node_n>)

    # NODES (macro multi-arg) takes the node names for which keys will be generated
    # SECURITY (cmake arg) if not define or OFF, will not generate key/keystores
    # ROS_SECURITY_ROOT_DIRECTORY (env variable) will the location of the keystore
    # POLICY_FILE (cmake arg) if defined, will compile policies by node name into the access private certificates (e.g POLICY_FILE=/etc/policies/<policy.yaml>, Generate: <node_name> /etc/policies/<policy.yaml>) **if defined, all nodes must have a policy defined for them**

