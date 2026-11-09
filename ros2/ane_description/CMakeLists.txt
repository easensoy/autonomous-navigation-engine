cmake_minimum_required(VERSION 3.16)
project(ane_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # Expands the Xacro and checks the result with check_urdf (urdfdom): one root, every joint between
  # existing links, valid inertias.
  find_program(XACRO xacro)
  find_program(CHECK_URDF check_urdf)
  if(XACRO AND CHECK_URDF)
    add_test(NAME ane_amr_urdf_valid
      COMMAND bash -c "set -e; ${XACRO} ${CMAKE_CURRENT_SOURCE_DIR}/urdf/ane_amr.urdf.xacro > ${CMAKE_CURRENT_BINARY_DIR}/ane_amr.urdf && ${CHECK_URDF} ${CMAKE_CURRENT_BINARY_DIR}/ane_amr.urdf")
    set_tests_properties(ane_amr_urdf_valid PROPERTIES ENVIRONMENT "AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}")
  endif()
endif()

ament_package()
