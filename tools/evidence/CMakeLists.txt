# Tools that regenerate the README's figures and measurements. Built only when the top-level
# project is configured with -DBUILD_EVIDENCE_TOOLS=ON; see tools/evidence/README.md.

add_library(evidence_common STATIC evidence_common.cpp)
target_link_libraries(evidence_common PUBLIC NavigationEngine)
target_include_directories(evidence_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Trace mode: attaches search observers and exports JSON; times nothing.
add_executable(evidence_export evidence_export.cpp)
target_link_libraries(evidence_export PRIVATE evidence_common)

# Measurement mode: no observers, console logging silenced, raw timings to CSV.
add_executable(evidence_bench evidence_bench.cpp)
target_link_libraries(evidence_bench PRIVATE evidence_common)
