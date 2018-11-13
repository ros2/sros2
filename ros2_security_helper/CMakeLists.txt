cmake_minimum_required(VERSION 3.1)
project(ros2_security_helper)
SET(VERSION "1.0.0")

include(CMakePackageConfigHelpers)
SET(LIB_INSTALL_DIR lib/)
SET(INCLUDE_INSTALL_DIR include/)
SET(SYSCONFIG_INSTALL_DIR share/${PROJECT_NAME})

configure_package_config_file(ros2_security_helperConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/ros2_security_helperConfig.cmake
    INSTALL_DESTINATION ${LIB_INSTALL_DIR}/ros2_security_helper/cmake

    PATH_VARS INCLUDE_INSTALL_DIR SYSCONFIG_INSTALL_DIR)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/ros2_security_helperConfigVersion.cmake
    VERSION ${VERSION}
    COMPATIBILITY SameMajorVersion )

INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake DESTINATION share/${PROJECT_NAME}/cmake)
INSTALL(FILES GenerateSecurity.cmake DESTINATION share/${PROJECT_NAME}/cmake)

