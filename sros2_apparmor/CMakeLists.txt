cmake_minimum_required(VERSION 3.5)
project(sros2_apparmor)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# TODO: install apparmor.d for linux targets and deb packaging
# install(
#   DIRECTORY apparmor.d
#   DESTINATION /etc/apparmor.d/
# )
