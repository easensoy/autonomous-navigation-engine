#include "pathfinding_algorithms/Dijkstra.hpp"
#include "dijkstra_internals.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <memory>
#include <unordered_map>

// Dijkstra core: construction, the single-source search loop and the public queries.
// Relaxation, priority-queue and shortest-path-tree helpers live in the other dijkstra/*.cpp files.

Dijkstra::Dijkstra(const Graph& environment) : Dijkstra(&environment) {}

Dijkstra::Dijkstra(const Graph* environment)
    : graph(environment),
      relaxationManager(std::make_unique<DistanceRelaxationManager>()),
      pqManager(std::make_unique<PriorityQueueManager>()),
      treeManager(nullptr) {
    if (!graph) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
}

// Special members defined here, where dijkstra_internals.hpp makes the helper classes complete.
Dijkstra::~Dijkstra() = default;
Dijkstra::Dijkstra(Dijkstra&& other) noexcept = default;
Dijkstra& Dijkstra::operator=(Dijkstra&& other) noexcept = default;

std::unordered_map<int, double> Dijkstra::findShortestDistances(int startId) {
    if (!graph->hasNode(startId)) {
        return {};  // unknown start: nothing is reachable
    }

    initializeDistanceRelaxation(graph->getAllNodeIds(), startId);
    initializePriorityQueue();
    clearShortestPathTree();  // a tree built for a previous source is no longer valid
    addToPriorityQueue(DijkstraNode(startId, 0.0));

    std::unordered_set<int> settled;
    while (!isPriorityQueueEmpty()) {
        int current = -1;
        try {
            current = getNextFromPriorityQueue().nodeId;
        } catch (const std::runtime_error&) {
            break;  // only superseded queue entries were left
        }
        if (!settled.insert(current).second) {
            continue;
        }

        for (const Edge& edge : graph->getEdgesFrom(current)) {
            if (edge.getWeight() < 0.0) {
                throw std::invalid_argument("Dijkstra requires non-negative edge weights");
            }
            const int neighbor = edge.getToNode();
            if (settled.count(neighbor) == 0 && relaxEdge(current, neighbor, edge.getWeight())) {
                addToPriorityQueue(DijkstraNode(neighbor, getCurrentDistance(neighbor), current));
            }
        }
    }
    return getAllCurrentDistances();
}

std::vector<int> Dijkstra::findShortestPath(int startId, int goalId) {
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        return {};  // unknown start or goal: no path (the test suite's contract)
    }
    findShortestDistances(startId);
    if (!isNodeReachable(goalId)) {
        return {};
    }
    return reconstructPath(getAllPredecessors(), startId, goalId);
}

std::vector<int> Dijkstra::reconstructPath(const std::unordered_map<int, int>& previousNodes, int startId,
                                           int goalId) const {
    std::vector<int> path{goalId};
    int current = goalId;
    while (current != startId) {
        auto it = previousNodes.find(current);
        if (it == previousNodes.end() || it->second == -1) {
            return {};
        }
        current = it->second;
        path.push_back(current);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

double Dijkstra::getShortestDistance(int startId, int goalId) {
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        return std::numeric_limits<double>::infinity();
    }
    findShortestDistances(startId);
    return getCurrentDistance(goalId);
}

bool Dijkstra::hasPath(int startId, int goalId) {
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        return false;
    }
    findShortestDistances(startId);
    return isNodeReachable(goalId);
}

void Dijkstra::printAlgorithmSteps(bool enable) {
    if (!relaxationManager) {
        relaxationManager = std::make_unique<DistanceRelaxationManager>();
    }
    relaxationManager->setLogging(enable);
}
