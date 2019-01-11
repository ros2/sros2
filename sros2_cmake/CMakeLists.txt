cmake_minimum_required(VERSION 3.1)
project(sros2_cmake)
SET(VERSION "1.0.0")

include(CMakePackageConfigHelpers)
SET(LIB_INSTALL_DIR lib/)
SET(INCLUDE_INSTALL_DIR include/)
SET(SYSCONFIG_INSTALL_DIR share/${PROJECT_NAME})

configure_package_config_file(sros2_cmakeConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/sros2_cmakeConfig.cmake
    INSTALL_DESTINATION ${LIB_INSTALL_DIR}/sros2_cmake/cmake

    PATH_VARS INCLUDE_INSTALL_DIR SYSCONFIG_INSTALL_DIR)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/sros2_cmakeConfigVersion.cmake
    VERSION ${VERSION}
    COMPATIBILITY SameMajorVersion )

INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES cmake/ros2_secure_node.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES cmake/ros2_create_keystore.cmake DESTINATION share/${PROJECT_NAME}/cmake)

