# Security Helper
Add authentication, cryptography, and access control security keys using a cmake macro.
The macro will generate the secure root directory if it does not exists, then create authentication and cryptography keys.

In package.xml add:  
`<depend>sros2_cmake</depend>`  
In CMakeLists add:  
`find_package(sros2_cmake REQUIRED)`  
`sros2_generate_artifacts(ENCLAVES <enclave_name>)`  

Macro definition:  
```
    # sros2_generate_artifacts(ENCLAVES <enclave_1> <enclave_2>...<enclave_n>)

    # ENCLAVES (macro multi-arg) takes the enclaves names for which keys will be generated
    #   Executables can use a different or the same enclaves.
    #   All nodes in the same process use the same enclave.
    # SECURITY (cmake arg) if not define or OFF, will not generate key/keystores
    # ROS_SECURITY_KEYSTORE (env variable) the location of the keystore
    # POLICY_FILE (cmake arg) if defined, will generate security artifacts for each enclave defined in the policy file.
```
