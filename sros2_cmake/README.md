# Security Helper
Add authentication, cryptography, and access control security keys using a cmake macro.
The macro will generate the secure root directory if it does not exists, then create authentication and cryptography keys.

In package.xml add:  
`<depend>sros2_cmake</depend>`  
In CMakeLists add:  
`find_package(sros2_cmake REQUIRED)`  
`sros2_generate_artifacts(SECURITY_CONTEXTS <context_name>)`  

Macro definition:  
```
    # sros2_generate_artifacts(SECURITY_CONTEXTS <context_1> <context_2>...<context_n>)

    # SECURITY_CONTEXTS (macro multi-arg) takes the security contexts names for which keys will be generated
    #   Executables can use a different or the same security contexts.
    #   All nodes in the same process use the same security context.
    # SECURITY (cmake arg) if not define or OFF, will not generate key/keystores
    # ROS_SECURITY_ROOT_DIRECTORY (env variable) the location of the keystore
    # POLICY_FILE (cmake arg) if defined, will generate security artifacts for each context defined in the policy file.
```
