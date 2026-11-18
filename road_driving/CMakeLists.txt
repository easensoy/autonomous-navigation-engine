# road_driving: synthetic left-hand-traffic road driving on top of the engine's Graph and NavigationCore.
# Configured from the repository root with -DBUILD_ROAD_DRIVING=ON; the default build is unchanged.
if(NOT TARGET NavigationEngine)
  message(FATAL_ERROR "Configure road_driving from the repository root with -DBUILD_ROAD_DRIVING=ON")
endif()

# Do not inherit the engine's global include path: only src/engine_router.cpp may see engine headers.
set_property(DIRECTORY PROPERTY INCLUDE_DIRECTORIES "")
get_filename_component(ROAD_ENGINE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

# The engine commit and whether tracked library sources (include/, src/) differ from it, at configure time.
set(ROAD_ENGINE_COMMIT "unknown")
set(ROAD_ENGINE_SOURCES_MODIFIED -1)
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
                  WORKING_DIRECTORY "${ROAD_ENGINE_ROOT}" OUTPUT_VARIABLE _road_commit
                  OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE _road_rc ERROR_QUIET)
  if(_road_rc EQUAL 0)
    set(ROAD_ENGINE_COMMIT "${_road_commit}")
    execute_process(COMMAND "${GIT_EXECUTABLE}" diff --quiet HEAD -- include src
                    WORKING_DIRECTORY "${ROAD_ENGINE_ROOT}" RESULT_VARIABLE _road_dirty ERROR_QUIET)
    if(_road_dirty EQUAL 0)
      set(ROAD_ENGINE_SOURCES_MODIFIED 0)
    else()
      set(ROAD_ENGINE_SOURCES_MODIFIED 1)
    endif()
  endif()
endif()

add_library(road_driving STATIC
    src/lane_network.cpp
    src/district.cpp
    src/drive_path.cpp
    src/vehicle.cpp
    src/engine_router.cpp
    src/stack.cpp
    src/episode.cpp
    src/evaluator.cpp
    src/sensors.cpp
    src/estimators.cpp
    src/benchmark.cpp
    src/live.cpp
)
target_include_directories(road_driving PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
                                        PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
set_source_files_properties(src/engine_router.cpp PROPERTIES INCLUDE_DIRECTORIES "${ROAD_ENGINE_ROOT}/include")
target_link_libraries(road_driving PRIVATE NavigationEngine)
target_compile_definitions(road_driving PRIVATE
    ROAD_ENGINE_COMMIT="${ROAD_ENGINE_COMMIT}"
    ROAD_ENGINE_SOURCES_MODIFIED=${ROAD_ENGINE_SOURCES_MODIFIED})
set_target_properties(road_driving PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)

add_executable(road_drive apps/road_drive.cpp)
target_link_libraries(road_drive PRIVATE road_driving)

add_executable(test_road_driving tests/test_road_driving.cpp)
target_link_libraries(test_road_driving PRIVATE road_driving)

add_executable(test_localisation tests/test_localisation.cpp)
target_link_libraries(test_localisation PRIVATE road_driving)

add_executable(test_live tests/test_live.cpp)
target_link_libraries(test_live PRIVATE road_driving)

add_executable(test_district tests/test_district.cpp)
target_link_libraries(test_district PRIVATE road_driving)

foreach(_road_target road_driving road_drive test_road_driving test_localisation test_live test_district)
  if(MSVC)
    target_compile_options(${_road_target} PRIVATE /W4)
  else()
    target_compile_options(${_road_target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endforeach()

enable_testing()
add_test(NAME road_driving_tests COMMAND test_road_driving)
add_test(NAME road_driving_localisation_tests COMMAND test_localisation)
# The live server behind the 3D viewer, and its recorded sessions (no graphics needed).
add_test(NAME road_driving_live_tests COMMAND test_live)
# The Kensington and Knightsbridge district networks: geometry, lane graph, feasible turning paths, routes.
add_test(NAME road_driving_district_tests COMMAND test_district)
# A session recorded through the 3D viewer (operator commands at recorded ticks) must replay headlessly:
# bit for bit on the recording toolchain, to the same outcome on another.
add_test(NAME road_driving_session_replay
         COMMAND road_drive session-replay "${CMAKE_CURRENT_SOURCE_DIR}/viewer/examples/sandbox_session.json")
# The stored failing episode (variant B0, notification 3 m before J10) must replay to the same outcome
# and route decisions; the trace hash must also match when the toolchain is the one that recorded it.
add_test(NAME road_driving_regression_replay
         COMMAND road_drive replay "${CMAKE_CURRENT_SOURCE_DIR}/regression/c4_d3_B0")
# The replay must also be able to fail: a copy with its variant label swapped has to be rejected.
add_test(NAME road_driving_replay_detects_change
         COMMAND "${CMAKE_COMMAND}" "-DROAD_DRIVE=$<TARGET_FILE:road_drive>"
                 "-DEPISODE=${CMAKE_CURRENT_SOURCE_DIR}/regression/c4_d3_B0"
                 "-DWORK=${CMAKE_CURRENT_BINARY_DIR}/replay_detects_change"
                 -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/replay_detects_change.cmake")
