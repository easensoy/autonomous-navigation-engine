cmake_minimum_required(VERSION 3.16)
project(AutonomousNavigationEngine VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(CORE_SOURCES
    src/core/node/node_constructor.cpp
    src/core/node/node_comparators.cpp
    src/core/node/euclidean_distance.cpp
    src/core/node/manhattan_distance.cpp
    src/core/edge/edge_constructor.cpp
    src/core/edge/edge_comparators.cpp
    src/core/edge/bidirectional_check.cpp
    src/core/edge/weight_validation.cpp
    src/core/graph/adjacency_list_builder.cpp
    src/core/graph/node_insertion.cpp
    src/core/graph/edge_insertion.cpp
    src/core/graph/graph_statistics.cpp
    src/core/graph/connectivity_checker.cpp
)

set(PATHFINDING_SOURCES
    src/pathfinding_algorithms/a_star/a_star_core.cpp
    src/pathfinding_algorithms/a_star/heuristic_functions.cpp
    src/pathfinding_algorithms/a_star/open_list_management.cpp
    src/pathfinding_algorithms/a_star/closed_list_management.cpp
    src/pathfinding_algorithms/dijkstra/dijkstra_core.cpp
    src/pathfinding_algorithms/dijkstra/distance_relaxation.cpp
    src/pathfinding_algorithms/dijkstra/priority_queue_ops.cpp
    src/pathfinding_algorithms/dijkstra/shortest_path_tree.cpp
    src/pathfinding_algorithms/bfs/bfs_traversal.cpp
    src/pathfinding_algorithms/bfs/unweighted_pathfinding.cpp
    src/pathfinding_algorithms/bfs/queue_operations.cpp
    src/pathfinding_algorithms/dfs/dfs_traversal.cpp
    src/pathfinding_algorithms/dfs/recursive_search.cpp
    src/pathfinding_algorithms/dfs/stack_operations.cpp
    src/pathfinding_algorithms/bellman_ford/bellman_ford_core.cpp
    src/pathfinding_algorithms/bellman_ford/negative_cycle_detection.cpp
    src/pathfinding_algorithms/bellman_ford/edge_relaxation.cpp
    src/pathfinding_algorithms/jump_point_search/jps_core.cpp
    src/pathfinding_algorithms/jump_point_search/jump_point_identification.cpp
    src/pathfinding_algorithms/jump_point_search/grid_pruning.cpp
)

set(NAVIGATION_SOURCES
    src/navigation_strategies/global_planning/complete_path_planning.cpp
    src/navigation_strategies/global_planning/global_optimization.cpp
    src/navigation_strategies/global_planning/waypoint_generation.cpp
    src/navigation_strategies/local_planning/dynamic_window_approach.cpp
    src/navigation_strategies/local_planning/obstacle_avoidance.cpp
    src/navigation_strategies/local_planning/potential_field_method.cpp
    src/navigation_strategies/dynamic_replanning/d_star_lite.cpp
    src/navigation_strategies/dynamic_replanning/incremental_search.cpp
    src/navigation_strategies/dynamic_replanning/real_time_replanning.cpp
    src/navigation_strategies/multi_goal/multi_destination_routing.cpp
    src/navigation_strategies/multi_goal/priority_based_planning.cpp
    src/navigation_strategies/multi_goal/traveling_salesman.cpp
)

set(GRAPH_OPERATIONS_SOURCES
    src/graph_operations/node_operations/add_node.cpp
    src/graph_operations/node_operations/remove_node.cpp
    src/graph_operations/node_operations/find_nearest_node.cpp
    src/graph_operations/node_operations/node_clustering.cpp
    src/graph_operations/edge_operations/add_edge.cpp
    src/graph_operations/edge_operations/remove_edge.cpp
    src/graph_operations/edge_operations/update_edge_weight.cpp
    src/graph_operations/edge_operations/edge_contraction.cpp
    src/graph_operations/graph_building/grid_graph_generator.cpp
    src/graph_operations/graph_building/random_graph_generator.cpp
    src/graph_operations/graph_building/delaunay_triangulation.cpp
    src/graph_operations/graph_building/visibility_graph_builder.cpp
    src/graph_operations/analysis/graph_diameter_calculation.cpp
    src/graph_operations/analysis/minimum_spanning_tree.cpp
    src/graph_operations/analysis/strongly_connected_components.cpp
)

set(PATH_OPERATIONS_SOURCES
    src/path_operations/validation/validate_path.cpp
    src/path_operations/validation/constraint_verification.cpp
    src/path_operations/validation/feasibility_analysis.cpp
    src/path_operations/validation/safety_checking.cpp
    src/path_operations/optimization/path_length_optimization.cpp
    src/path_operations/optimization/time_optimal_planning.cpp
    src/path_operations/optimization/curvature_optimization.cpp
    src/path_operations/smoothing/bezier_curve_smoothing.cpp
    src/path_operations/smoothing/spline_interpolation.cpp
    src/path_operations/smoothing/corner_cutting.cpp
    src/path_operations/cost_calculation/calculate_path_cost.cpp
    src/path_operations/cost_calculation/multi_criteria_costing.cpp
    src/path_operations/cost_calculation/dynamic_cost_updates.cpp
)

set(ENVIRONMENT_SOURCES
    src/environment/environment_management/set_environment.cpp
    src/environment/environment_management/environment_persistence.cpp
    src/environment/environment_management/environment_switching.cpp
    src/environment/map_updates/incremental_map_update.cpp
    src/environment/map_updates/map_synchronization.cpp
    src/environment/map_updates/sensor_data_integrationc.cpp
    src/environment/obstacle_detection/static_obstacle_detection.cpp
    src/environment/obstacle_detection/dynamic_obstacle_tracking.cpp
    src/environment/obstacle_detection/collision_prediction.cpp
)

set(AUTONOMOUS_NAVIGATOR_SOURCES
    src/autonomous_navigator/navigation_core/navigator_initialization.cpp
    src/autonomous_navigator/navigation_core/algorithm_execution.cpp
    src/autonomous_navigator/navigation_core/path_following.cpp
    src/autonomous_navigator/algorithm_selection/environment_based_selection.cpp
    src/autonomous_navigator/algorithm_selection/performance_based_selection.cpp
    src/autonomous_navigator/algorithm_selection/hybrid_algorithm_switching.cpp
    src/autonomous_navigator/execution/path_execution_controller.cpp
    src/autonomous_navigator/execution/real_time_navigation.cpp
    src/autonomous_navigator/execution/emergency_stop_handler.cpp
)

set(UTILITIES_SOURCES
    src/utilities/math/vector_operations.cpp
    src/utilities/math/angle_calculations.cpp
    src/utilities/math/interpolation_functions.cpp
    src/utilities/configuration/json_config_loader.cpp
    src/utilities/configuration/yaml_config_loader.cpp
    src/utilities/configuration/parameter_validation.cpp
    src/utilities/data_structures/binary_heap.cpp
    src/utilities/data_structures/fibonacci_heap.cpp
    src/utilities/data_structures/disjoint_set.cpp
)

set(REPORTING_SOURCES
    src/reporting/logging/navigation_event_logger.cpp
    src/reporting/logging/algorithm_step_logger.cpp
    src/reporting/logging/error_reporting.cpp
    src/reporting/performance/algorithm_benchmarking.cpp
    src/reporting/performance/execution_time_analysis.cpp
    src/reporting/performance/memory_usage_profiling.cpp
    src/reporting/visualization/ascii_graph_renderer.cpp
    src/reporting/visualization/console_path_printer.cpp
    src/reporting/visualization/svg_export.cpp
)

set(ALL_SOURCES
    ${CORE_SOURCES}
    ${PATHFINDING_SOURCES}
    ${NAVIGATION_SOURCES}
    ${GRAPH_OPERATIONS_SOURCES}
    ${PATH_OPERATIONS_SOURCES}
    ${ENVIRONMENT_SOURCES}
    ${AUTONOMOUS_NAVIGATOR_SOURCES}
    ${UTILITIES_SOURCES}
    ${REPORTING_SOURCES}
)

add_library(NavigationEngine STATIC ${ALL_SOURCES})

set_target_properties(NavigationEngine PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)
    add_executable(AutonomousNavigationEngine src/main.cpp)
    target_link_libraries(AutonomousNavigationEngine NavigationEngine)
endif()

option(BUILD_TESTS "Build test programs" ON)

if(BUILD_TESTS)
    enable_testing()
    
    set(TEST_SOURCES
        tests/algorithm_tests/test_a_star.cpp
        tests/algorithm_tests/test_dijkstra.cpp
        tests/algorithm_tests/test_bfs.cpp
        tests/algorithm_tests/test_bellman_ford.cpp
        tests/integration_tests/test_full_navigation.cpp
        tests/integration_tests/test_environment_switching.cpp
        tests/performance_tests/algorithm_benchmarks.cpp
        tests/performance_tests/scability_tests.cpp
    )
    
    foreach(TEST_SOURCE ${TEST_SOURCES})
        get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
        add_executable(${TEST_NAME} ${TEST_SOURCE})
        target_link_libraries(${TEST_NAME} NavigationEngine)
        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endforeach()
endif()

install(TARGETS NavigationEngine
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

set(CPACK_PACKAGE_NAME "AutonomousNavigationEngine")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Advanced pathfinding and navigation library")
include(CPack)