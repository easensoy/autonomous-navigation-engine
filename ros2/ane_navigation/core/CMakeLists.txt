# Adapter core: the ROS-independent part of the ROS 2 integration (roadmap, blockage mapping, segment
# controller, mission executive around the engine's NavigationCore), its tests and ane_route_check.
#
# Two ways to build it:
#   * standalone, without ROS (Windows, Linux, CI):
#       cmake -S ros2/ane_navigation/core -B build/adapter-core && cmake --build build/adapter-core
#       ctest --test-dir build/adapter-core --output-on-failure
#     This adds the engine from the repository root itself.
#   * inside the ane_navigation ROS 2 package, whose CMakeLists.txt adds the engine first.
cmake_minimum_required(VERSION 3.16)
project(ane_adapter_core LANGUAGES CXX)

get_filename_component(ANE_ENGINE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
get_filename_component(ANE_SIM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../ane_sim" ABSOLUTE)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release)
  endif()
  set(BUILD_TESTS OFF)  # the engine's own test programs
  add_subdirectory(${ANE_ENGINE_ROOT} engine EXCLUDE_FROM_ALL)
  option(ANE_CORE_TESTS "Build the adapter-core tests" ON)
endif()

if(NOT TARGET NavigationEngine)
  message(FATAL_ERROR "ane_adapter_core needs the engine's NavigationEngine target")
endif()

add_library(ane_adapter_core STATIC
  src/geometry.cpp
  src/roadmap.cpp
  src/blockage_mapper.cpp
  src/segment_controller.cpp
  src/mission_executive.cpp
)
target_include_directories(ane_adapter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${ANE_ENGINE_ROOT}/include>
)
target_compile_features(ane_adapter_core PUBLIC cxx_std_17)
target_compile_definitions(ane_adapter_core PUBLIC _USE_MATH_DEFINES)
target_link_libraries(ane_adapter_core PUBLIC NavigationEngine)
set_target_properties(ane_adapter_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(MSVC)
  target_compile_options(ane_adapter_core PRIVATE /W4)
else()
  target_compile_options(ane_adapter_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(ane_route_check tools/route_check.cpp)
target_link_libraries(ane_route_check PRIVATE ane_adapter_core)

if(ANE_CORE_TESTS)
  enable_testing()
  foreach(test_name test_adapter_core test_closed_loop)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE ane_adapter_core)
    target_compile_definitions(${test_name} PRIVATE
      ANE_ROADMAP_FILE="${ANE_SIM_DIR}/maps/warehouse_roadmap.txt"
      ANE_SCENARIO_DIR="${ANE_SIM_DIR}/scenarios")
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
