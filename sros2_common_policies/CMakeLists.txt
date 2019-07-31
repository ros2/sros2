cmake_minimum_required(VERSION 3.5)
project(sros2_common_policies)

find_package(ament_cmake REQUIRED)
find_package(sros2_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

sros2_cmake_install_policies(
  policies/node.xml
  policies/node/logging.xml
  policies/node/parameters.xml
  policies/node/time.xml
)

ament_package(
)
