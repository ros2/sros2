cmake_minimum_required(VERSION 3.1)
project(sros2_cmake)
set(VERSION "1.0.0")

include(CMakePackageConfigHelpers)
set(LIB_INSTALL_DIR lib/)
set(INCLUDE_INSTALL_DIR include/)
set(SYSCONFIG_INSTALL_DIR share/${PROJECT_NAME})

# ament_cmake_test provides BUILD_TESTING definition
find_package(ament_cmake_test REQUIRED)
if(BUILD_TESTING)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
endif()

configure_package_config_file(sros2_cmakeConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/sros2_cmakeConfig.cmake
  INSTALL_DESTINATION ${LIB_INSTALL_DIR}/sros2_cmake/cmake
  PATH_VARS INCLUDE_INSTALL_DIR SYSCONFIG_INSTALL_DIR
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/sros2_cmakeConfigVersion.cmake
  VERSION ${VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake DESTINATION share/${PROJECT_NAME}/cmake)
install(FILES ${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake DESTINATION share/${PROJECT_NAME}/cmake)
install(FILES cmake/ros2_secure_node.cmake DESTINATION share/${PROJECT_NAME}/cmake)
install(FILES cmake/ros2_create_keystore.cmake DESTINATION share/${PROJECT_NAME}/cmake)
