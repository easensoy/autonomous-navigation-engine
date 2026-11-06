#include "autonomous_navigator/AlgorithmSelector.hpp"
#include <stdexcept>

// AlgorithmSelector construction. Selection, execution and benchmarking live in the other
// algorithm_selection/*.cpp files.

AlgorithmSelector::AlgorithmSelector(const Graph* graph) : environment(graph) {
    if (!environment) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
}
