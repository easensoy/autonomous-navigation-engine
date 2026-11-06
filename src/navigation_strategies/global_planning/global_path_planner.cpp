#include "navigation_strategies/GlobalPathPlanner.hpp"
#include "pathfinding_algorithms/BellmanFord.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <memory>

// GlobalPathPlanner chooses between A*, Dijkstra and Bellman-Ford for whole-route planning.

GlobalPathPlanner::GlobalPathPlanner(const Graph* graph)
    : environment(graph), currentAlgorithm(PlanningAlgorithm::AUTO_SELECT) {
    if (!environment) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
    aStarPlanner = std::make_unique<AStar>(environment);
    dijkstraPlanner = std::make_unique<Dijkstra>(environment);
}

PlanningAlgorithm GlobalPathPlanner::selectOptimalAlgorithm(int /*startId*/, int /*goalId*/) const {
    // Negative weights rule out A* and Dijkstra; otherwise A* with its Euclidean heuristic.
    for (int id : environment->getAllNodeIds()) {
        for (const Edge& edge : environment->getEdgesFrom(id)) {
            if (edge.getWeight() < 0.0) {
                return PlanningAlgorithm::BELLMAN_FORD;
            }
        }
    }
    return PlanningAlgorithm::ASTAR;
}

std::vector<int> GlobalPathPlanner::planPath(int startId, int goalId) {
    return planPathWithAlgorithm(startId, goalId, currentAlgorithm);
}

std::vector<int> GlobalPathPlanner::planPathWithAlgorithm(int startId, int goalId, PlanningAlgorithm algorithm) {
    if (!environment->hasNode(startId) || !environment->hasNode(goalId)) {
        return {};
    }
    if (algorithm == PlanningAlgorithm::AUTO_SELECT) {
        algorithm = selectOptimalAlgorithm(startId, goalId);
    }
    switch (algorithm) {
        case PlanningAlgorithm::DIJKSTRA:
            return dijkstraPlanner->findShortestPath(startId, goalId);
        case PlanningAlgorithm::BELLMAN_FORD: {
            BellmanFord bellmanFord(environment);
            return bellmanFord.findShortestPath(startId, goalId);
        }
        case PlanningAlgorithm::ASTAR:
        case PlanningAlgorithm::AUTO_SELECT:
        default:
            return aStarPlanner->findPath(startId, goalId);
    }
}

std::vector<std::vector<int>> GlobalPathPlanner::planMultipleGoals(int startId, const std::vector<int>& goals) {
    std::vector<std::vector<int>> paths;
    paths.reserve(goals.size());
    for (int goal : goals) {
        paths.push_back(planPath(startId, goal));
    }
    return paths;
}

void GlobalPathPlanner::setAlgorithm(PlanningAlgorithm algorithm) {
    currentAlgorithm = algorithm;
}

PlanningAlgorithm GlobalPathPlanner::getCurrentAlgorithm() const {
    return currentAlgorithm;
}

double GlobalPathPlanner::estimatePathCost(int startId, int goalId) const {
    if (!environment->hasNode(startId) || !environment->hasNode(goalId)) {
        return std::numeric_limits<double>::infinity();
    }
    return dijkstraPlanner->getShortestDistance(startId, goalId);
}

void GlobalPathPlanner::optimizeForSpeed() {
    currentAlgorithm = PlanningAlgorithm::ASTAR;
}

void GlobalPathPlanner::optimizeForAccuracy() {
    currentAlgorithm = PlanningAlgorithm::DIJKSTRA;
}

void GlobalPathPlanner::enableAutoSelection(bool enable) {
    currentAlgorithm = enable ? PlanningAlgorithm::AUTO_SELECT : PlanningAlgorithm::ASTAR;
}
