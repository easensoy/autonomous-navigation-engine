#include "path_operations/PathOptimizer.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_map>

// PathOptimizer improves a route without leaving the graph: it removes loops and replaces a run
// of nodes with a direct edge when that edge is cheaper. A route never gets more expensive.

namespace {
double routeCost(const Graph& graph, const std::vector<int>& path) {
    double cost = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        if (!graph.hasEdge(path[i - 1], path[i])) {
            return std::numeric_limits<double>::infinity();
        }
        cost += graph.getEdgeWeight(path[i - 1], path[i]);
    }
    return cost;
}
}  // namespace

PathOptimizer::PathOptimizer(const Graph* graph)
    : environment(graph), currentCriteria(OptimizationCriteria::MINIMIZE_DISTANCE) {
    if (!environment) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
}

std::vector<int> PathOptimizer::removeRedundantWaypoints(const std::vector<int>& path) {
    // Cut loops: if a node appears again later, skip everything between the two visits.
    std::vector<int> result;
    std::unordered_map<int, size_t> position;
    for (int node : path) {
        auto seen = position.find(node);
        if (seen != position.end()) {
            for (size_t k = seen->second + 1; k < result.size(); ++k) {
                position.erase(result[k]);
            }
            result.resize(seen->second + 1);
            continue;
        }
        position[node] = result.size();
        result.push_back(node);
    }
    return result;
}

std::vector<int> PathOptimizer::shortcutPath(const std::vector<int>& path) {
    if (path.size() < 3) {
        return path;
    }
    std::vector<int> result{path.front()};
    size_t i = 0;
    while (i + 1 < path.size()) {
        // Furthest node reachable by one direct edge that is cheaper than following the route.
        size_t next = i + 1;
        double costAlongRoute = 0.0;
        for (size_t j = i + 1; j < path.size(); ++j) {
            costAlongRoute += environment->getEdgeWeight(path[j - 1], path[j]);
            if (j > i + 1 && environment->hasEdge(path[i], path[j]) &&
                environment->getEdgeWeight(path[i], path[j]) < costAlongRoute) {
                next = j;
            }
        }
        result.push_back(path[next]);
        i = next;
    }
    return result;
}

std::vector<int> PathOptimizer::optimizeForDistance(const std::vector<int>& originalPath) {
    std::vector<int> path = removeRedundantWaypoints(originalPath);
    if (routeCost(*environment, path) == std::numeric_limits<double>::infinity()) {
        return originalPath;  // not a connected route; leave it unchanged
    }
    return shortcutPath(path);
}

std::vector<int> PathOptimizer::optimizeForTime(const std::vector<int>& originalPath) {
    // No speed model is stored on edges, so travel time is proportional to edge cost.
    return optimizeForDistance(originalPath);
}

std::vector<int> PathOptimizer::optimizePath(const std::vector<int>& originalPath) {
    return optimizeWithCriteria(originalPath, currentCriteria);
}

std::vector<int> PathOptimizer::optimizeWithCriteria(const std::vector<int>& originalPath,
                                                     OptimizationCriteria criteria) {
    if (originalPath.size() < 3) {
        return originalPath;
    }
    return criteria == OptimizationCriteria::MINIMIZE_TIME ? optimizeForTime(originalPath)
                                                           : optimizeForDistance(originalPath);
}

std::vector<int> PathOptimizer::optimizeMultiObjective(const std::vector<int>& originalPath,
                                                       const std::vector<double>& /*weights*/) {
    // Distance is the only objective with data behind it on this graph.
    return optimizeForDistance(originalPath);
}

void PathOptimizer::setOptimizationCriteria(OptimizationCriteria criteria) {
    currentCriteria = criteria;
}

double PathOptimizer::calculateOptimizationImprovement(const std::vector<int>& originalPath,
                                                       const std::vector<int>& optimizedPath) {
    const double before = routeCost(*environment, originalPath);
    const double after = routeCost(*environment, optimizedPath);
    if (before <= 0.0 || before == std::numeric_limits<double>::infinity() ||
        after == std::numeric_limits<double>::infinity()) {
        return 0.0;
    }
    return (before - after) / before * 100.0;  // percent saved
}

std::vector<int> PathOptimizer::interpolateWaypoints(const std::vector<int>& sparsePath, int /*maxSegmentLength*/) {
    // Fill gaps between consecutive waypoints that are not adjacent with a shortest route.
    if (sparsePath.size() < 2) {
        return sparsePath;
    }
    AStar search(environment);
    std::vector<int> dense{sparsePath.front()};
    for (size_t i = 1; i < sparsePath.size(); ++i) {
        if (environment->hasEdge(sparsePath[i - 1], sparsePath[i])) {
            dense.push_back(sparsePath[i]);
            continue;
        }
        std::vector<int> leg = search.findPath(sparsePath[i - 1], sparsePath[i]);
        if (leg.empty()) {
            return {};
        }
        dense.insert(dense.end(), leg.begin() + 1, leg.end());
    }
    return dense;
}

bool PathOptimizer::isOptimizationNecessary(const std::vector<int>& path) const {
    if (path.size() < 3) {
        return false;
    }
    auto* self = const_cast<PathOptimizer*>(this);  // the helpers are non-const but do not modify state
    return self->optimizeWithCriteria(path, currentCriteria) != path;
}
