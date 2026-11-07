#include "pathfinding_algorithms/AStar.hpp"
#include <cmath>
#include <algorithm>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

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

double AStar::minimumCostPerLength() const {
    // One pass over the edges, repeated only after the graph changes.
    if (costPerLengthKnown && costPerLengthRevision == graph->getRevision()) {
        return costPerLength;
    }
    // For every edge u -> v, scale * |uv| <= cost(u, v). With the triangle inequality this gives
    // h(u) <= cost(u, v) + h(v) for h = scale * straight-line distance: consistent, so admissible.
    double scale = std::numeric_limits<double>::infinity();
    for (int nodeId : graph->getAllNodeIds()) {
        const Node& from = graph->getNode(nodeId);
        for (const Edge& edge : graph->getEdgesFrom(nodeId)) {
            if (edge.getWeight() < 0.0) {
                throw std::invalid_argument("A* requires non-negative edge weights");
            }
            const double length = from.euclideanDistance(graph->getNode(edge.getToNode()));
            if (length > 0.0) {
                scale = std::min(scale, edge.getWeight() / length);
            }
        }
    }
    costPerLength = std::isfinite(scale) ? scale : 0.0;  // no edge with a length: h = 0, as in Dijkstra
    costPerLengthRevision = graph->getRevision();
    costPerLengthKnown = true;
    return costPerLength;
}

bool AStar::isHeuristicAdmissible(int startId, int goalId) const {
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        return false;
    }
    const double scale = minimumCostPerLength();
    auto heuristic = [this, scale](int nodeId, int goal) {
        return heuristicFunction ? heuristicFunction(nodeId, goal) : scale * calculateHeuristic(nodeId, goal);
    };

    // Exact cheapest cost from every node to the goal: Dijkstra over the reversed edges.
    std::unordered_map<int, std::vector<std::pair<int, double>>> incoming;
    for (int nodeId : graph->getAllNodeIds()) {
        for (const Edge& edge : graph->getEdgesFrom(nodeId)) {
            incoming[edge.getToNode()].emplace_back(nodeId, edge.getWeight());
        }
    }
    std::unordered_map<int, double> costToGoal{{goalId, 0.0}};
    using Entry = std::pair<double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    queue.emplace(0.0, goalId);
    while (!queue.empty()) {
        const auto [cost, node] = queue.top();
        queue.pop();
        if (cost > costToGoal.at(node)) {
            continue;
        }
        const auto edgesIn = incoming.find(node);
        if (edgesIn == incoming.end()) {
            continue;
        }
        for (const auto& [previous, weight] : edgesIn->second) {
            const double candidate = cost + weight;
            const auto known = costToGoal.find(previous);
            if (known == costToGoal.end() || candidate < known->second) {
                costToGoal[previous] = candidate;
                queue.emplace(candidate, previous);
            }
        }
    }

    // Every node a search from startId can reach must be estimated at or below its true cost.
    std::unordered_set<int> reached{startId};
    std::vector<int> frontier{startId};
    while (!frontier.empty()) {
        const int node = frontier.back();
        frontier.pop_back();
        const auto known = costToGoal.find(node);
        if (known != costToGoal.end() && heuristic(node, goalId) > known->second * (1.0 + 1e-9) + 1e-12) {
            return false;
        }
        for (const Edge& edge : graph->getEdgesFrom(node)) {
            if (reached.insert(edge.getToNode()).second) {
                frontier.push_back(edge.getToNode());
            }
        }
    }
    return true;
}