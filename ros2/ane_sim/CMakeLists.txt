cmake_minimum_required(VERSION 3.16)
project(ane_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config maps worlds scenarios DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/scenario_runner.py
  scripts/trajectory_path.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/scenario_lib.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  # The committed world and roadmap must match the layout they are generated from.
  add_test(NAME world_and_roadmap_up_to_date
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_world.py --check)
  # The world is valid SDFormat.
  find_program(GZ gz)
  if(GZ)
    add_test(NAME warehouse_world_valid_sdf COMMAND ${GZ} sdf -k ${CMAKE_CURRENT_SOURCE_DIR}/worlds/warehouse.sdf)
  endif()
endif()

ament_package()
