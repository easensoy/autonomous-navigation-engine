#include "pathfinding_algorithms/AStar.hpp"
#include <cmath>
#include <algorithm>

double AStar::calculateHeuristic(int nodeId, int goalId) const {
    if (!graph->hasNode(nodeId) || !graph->hasNode(goalId)) {
        return std::numeric_limits<double>::infinity();
    }
    
    const Node& current = graph->getNode(nodeId);
    const Node& goal = graph->getNode(goalId);
    
    return current.euclideanDistance(goal);
}

void AStar::setHeuristicFunction(std::function<double(int, int)> heuristic) {
    if (!heuristic) {
        throw std::invalid_argument("Heuristic function cannot be null");
    }
    heuristicFunction = heuristic;
}

std::vector<int> AStar::findPathWithHeuristic(int startId, int goalId, 
                                            std::function<double(int, int)> customHeuristic) {
    if (!customHeuristic) {
        throw std::invalid_argument("Custom heuristic function cannot be null");
    }
    
    auto originalHeuristic = heuristicFunction;
    heuristicFunction = customHeuristic;
    
    std::vector<int> result = findPath(startId, goalId);
    
    heuristicFunction = originalHeuristic;
    return result;
}

double AStar::manhattanHeuristic(int nodeId, int goalId) const {
    if (!graph->hasNode(nodeId) || !graph->hasNode(goalId)) {
        return std::numeric_limits<double>::infinity();
    }
    
    const Node& current = graph->getNode(nodeId);
    const Node& goal = graph->getNode(goalId);
    
    return current.manhattanDistance(goal);
}

double AStar::euclideanHeuristic(int nodeId, int goalId) const {
    return calculateHeuristic(nodeId, goalId);
}

double AStar::chebyshevHeuristic(int nodeId, int goalId) const {
    if (!graph->hasNode(nodeId) || !graph->hasNode(goalId)) {
        return std::numeric_limits<double>::infinity();
    }
    
    const Node& current = graph->getNode(nodeId);
    const Node& goal = graph->getNode(goalId);
    
    double dx = std::abs(current.getX() - goal.getX());
    double dy = std::abs(current.getY() - goal.getY());
    
    return std::max(dx, dy) * 0.1; // Scale to match edge weights
}

bool AStar::isHeuristicAdmissible(int startId, int goalId) const {
    std::vector<int> actualPath = findPath(startId, goalId);
    if (actualPath.empty()) {
        return true; // No path exists, heuristic is trivially admissible
    }
    
    double actualCost = getPathCost(actualPath);
    double heuristicEstimate = heuristicFunction(startId, goalId);
    
    return heuristicEstimate <= actualCost * 1.01; // Allow small numerical tolerance
}