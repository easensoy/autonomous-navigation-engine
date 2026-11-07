#include "pathfinding_algorithms/AStar.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <memory>
#include <string>
#include <unordered_map>

// A* search core: construction, the search loop, path reconstruction and search statistics.
// The open/closed list helpers live in open_list_management.cpp / closed_list_management.cpp
// and the heuristics in heuristic_functions.cpp.

AStar::AStarNode::AStarNode(int id, double g, double f, int p)
    : nodeId(id), gScore(g), fScore(f), parent(p) {}

bool AStar::AStarNode::operator>(const AStarNode& other) const {
    if (fScore != other.fScore) {
        return fScore > other.fScore;
    }
    return gScore < other.gScore;  // on equal f, prefer the node further from the start
}

AStar::AStar(const Graph* environment)
    : graph(environment),
      openListManager(std::make_unique<OpenListManager>()),
      closedListManager(std::make_unique<ClosedListManager>()),
      nodesExplored(0),
      lastSearchTime(0.0),
      searchStatisticsEnabled(false) {
    if (!graph) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
}

std::vector<int> AStar::findPath(int startId, int goalId) const {
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        return {};  // unknown start or goal: no path (the test suite's contract)
    }

    const auto searchStart = std::chrono::steady_clock::now();
    openListManager->clear();
    closedListManager->clear();
    nodesExplored = 0;

    // The default heuristic is straight-line distance scaled by the smallest cost per unit length of
    // any edge, so it never overestimates and A* stays optimal whatever the edge costs are.
    // minimumCostPerLength() also rejects negative weights, which A* cannot handle.
    const double scale = minimumCostPerLength();
    auto heuristic = [this, scale](int nodeId, int goal) {
        return heuristicFunction ? heuristicFunction(nodeId, goal) : scale * calculateHeuristic(nodeId, goal);
    };

    std::unordered_map<int, double> gScore;
    std::unordered_map<int, int> cameFrom;
    gScore[startId] = 0.0;
    openListManager->push(AStarNode(startId, 0.0, heuristic(startId, goalId), -1));

    std::vector<int> path;
    while (!openListManager->empty()) {
        AStarNode current = openListManager->pop();
        // Skip entries made stale by a cheaper route, and nodes already expanded at this cost. A
        // closed node is expanded again only when a strictly cheaper route reaches it, which a custom
        // heuristic that is admissible but not consistent can cause.
        const auto best = gScore.find(current.nodeId);
        if ((best != gScore.end() && current.gScore > best->second) ||
            (closedListManager->contains(current.nodeId) &&
             current.gScore >= closedListManager->getGScore(current.nodeId))) {
            continue;
        }
        closedListManager->add(current.nodeId, current.gScore, current.fScore);
        ++nodesExplored;

        if (current.nodeId == goalId) {
            path = reconstructPath(cameFrom, goalId);
            break;
        }

        for (const Edge& edge : graph->getEdgesFrom(current.nodeId)) {
            const int neighbor = edge.getToNode();
            const double tentative = current.gScore + edge.getWeight();
            auto known = gScore.find(neighbor);
            if (known == gScore.end() || tentative < known->second) {
                gScore[neighbor] = tentative;
                cameFrom[neighbor] = current.nodeId;
                openListManager->push(AStarNode(neighbor, tentative, tentative + heuristic(neighbor, goalId),
                                                current.nodeId));
            }
        }
    }

    lastSearchTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - searchStart).count();
    updateSearchStatistics();
    return path;
}

std::vector<int> AStar::reconstructPath(const std::unordered_map<int, int>& cameFrom, int current) const {
    std::vector<int> path{current};
    auto it = cameFrom.find(current);
    while (it != cameFrom.end()) {
        current = it->second;
        path.push_back(current);
        it = cameFrom.find(current);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

double AStar::getPathCost(const std::vector<int>& path) const {
    double cost = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        if (!graph->hasEdge(path[i - 1], path[i])) {
            throw std::runtime_error("Path contains an invalid edge: " + std::to_string(path[i - 1]) + " -> " +
                                     std::to_string(path[i]));
        }
        cost += graph->getEdgeWeight(path[i - 1], path[i]);
    }
    return cost;
}

void AStar::updateSearchStatistics() const {
    if (searchStatisticsEnabled) {
        printSearchStatistics();
    }
}

void AStar::printSearchStatistics() const {
    std::cout << "[A*] Last search: " << nodesExplored << " nodes explored in " << lastSearchTime * 1000.0
              << " ms" << std::endl;
}

size_t AStar::getLastSearchNodesExplored() const {
    return nodesExplored;
}

double AStar::getLastSearchExecutionTime() const {
    return lastSearchTime;
}

void AStar::enableSearchStatistics(bool enable) {
    searchStatisticsEnabled = enable;
}

std::vector<int> AStar::getLastSearchExploredNodes() const {
    return getExploredNodes();
}

size_t AStar::estimateMemoryUsage() const {
    const size_t perNode = sizeof(AStarNode) + 4 * sizeof(int) + 2 * sizeof(double);
    return sizeof(AStar) + (getOpenListSize() + getClosedListSize()) * perNode;
}

void AStar::resetSearchState() {
    clearSearchState();
    nodesExplored = 0;
    lastSearchTime = 0.0;
}
