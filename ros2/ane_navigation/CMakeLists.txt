cmake_minimum_required(VERSION 3.16)
project(ane_navigation LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ane_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# This repository's engine (repository root) and the ROS-independent adapter core. The engine is built
# from source here, so every node links the planner code of this checkout.
get_filename_component(ANE_ENGINE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(BUILD_TESTS OFF)  # the engine's own test programs are built and run by the repository's CMake/CI
add_subdirectory(${ANE_ENGINE_ROOT} ${CMAKE_CURRENT_BINARY_DIR}/engine EXCLUDE_FROM_ALL)
set(ANE_CORE_TESTS ${BUILD_TESTING})
add_subdirectory(core ${CMAKE_CURRENT_BINARY_DIR}/core)

set(node_dependencies rclcpp ane_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs std_srvs
    visualization_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(graph_navigator src/graph_navigator_node.cpp)
target_link_libraries(graph_navigator ane_adapter_core)
ament_target_dependencies(graph_navigator ${node_dependencies})

add_executable(cmd_vel_gate src/cmd_vel_gate_node.cpp)
ament_target_dependencies(cmd_vel_gate rclcpp geometry_msgs std_msgs)

add_executable(fault_injector src/fault_injector_node.cpp)
ament_target_dependencies(fault_injector rclcpp ane_interfaces geometry_msgs nav_msgs)

install(TARGETS graph_navigator cmd_vel_gate fault_injector ane_route_check
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
