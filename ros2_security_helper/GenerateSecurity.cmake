# Macro for setting up security

macro(ros2_create_keystore)
    IF (NOT SECURITY)
        return()
    endif()
    find_program(PROGRAM ros2)
    if (DEFINED ENV{ROS_SECURITY_ROOT_DIRECTORY})
        set(SECURITY_KEYSTORE $ENV{ROS_SECURITY_ROOT_DIRECTORY})
    else()
        SET(SECURITY_KEYSTORE ${DEFAULT_KEYSTORE})
    endif()
    message(STATUS "Keystore located at ${SECURITY_KEYSTORE}")
    IF (NOT EXISTS ${SECURITY_KEYSTORE})
        message(STATUS "Creating keystore directory")
        file(MAKE_DIRECTORY ${SECURITY_KEYSTORE})
    endif()

    # Check to see if the security keystore already has already been created
    file(GLOB RESULT "${SECURITY_KEYSTORE}/")
    list(LENGTH RESULT RES_LEN)
    if(${RES_LEN} EQUAL 0)
        message(STATUS "Creating keystore directory")
        execute_process (
	        COMMAND ${PROGRAM} security create_keystore ${SECURITY_KEYSTORE}
	    )
    endif()
endmacro()

macro(ros2_secure_node)
    # ros2_secure_node(NODES <node_1> <node_2>...<node_n>)

    # NODES (macro multi-arg) takes the node names for which keys will be generated
    # SECURITY (cmake arg) if not define or OFF, will not generate key/keystores
    # ROS_SECURITY_ROOT_DIRECTORY (env variable) will the location of the keystore
    # POLICY_FILE (cmake arg) if defined, will compile policies by node name into the access private certificates (e.g POLICY_FILE=/etc/policies/<policy.yaml>, Generate: <node_name> /etc/policies/<policy.yaml>)
    IF (NOT SECURITY)
        message(STATUS "Not generating security files")
        return()
    endif()
    find_program(PROGRAM ros2)
    if (NOT PROGRAM)
        message("Unable to find ros2cli, have you sourced your ros setup files?")
        return()
    endif()
    ros2_create_keystore()
    set(multiValueArgs NODES)
    cmake_parse_arguments(ros2_secure_node "" "" "${multiValueArgs}" ${ARGN} )
    foreach(node ${ros2_secure_node_NODES})
        message(STATUS "${PROGRAM} security create_key ${SECURITY_KEYSTORE} ${node} ${policy}")
        execute_process (
            COMMAND ${PROGRAM} security create_key ${SECURITY_KEYSTORE} ${node} 
        )
        if (POLICY_FILE)
            if (EXISTS ${POLICY_FILE})
                set(policy ${POLICY_FILE})
                message(STATUS "Executing: ${PROGRAM} security create_permission ${SECURITY_KEYSTORE} ${node} ${policy}")
                execute_process (
                    COMMAND ${PROGRAM} security create_permission ${SECURITY_KEYSTORE} ${node} ${policy}
                    RESULT_VARIABLE POLICY_RESULT
                    ERROR_VARIABLE POLICY_ERROR
                    )
                if (NOT ${POLICY_RESULT} EQUAL 0)
                    message("Unable to generate policy for ${node} in ${policy}")
                    message("${POLICY_ERROR}")
                endif()
            endif()
        endif()
    endforeach(node)
endmacro()

