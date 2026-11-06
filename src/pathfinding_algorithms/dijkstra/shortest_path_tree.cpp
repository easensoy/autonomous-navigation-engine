#include "pathfinding_algorithms/Dijkstra.hpp"
#include "dijkstra_internals.hpp"
#include <iostream>
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>

void Dijkstra::buildShortestPathTree(int startNode) {
    if (!relaxationManager) {
        throw std::runtime_error("Cannot build tree without completed distance calculation");
    }
    
    if (!treeManager) {
        treeManager = std::make_unique<ShortestPathTreeManager>();
    }
    
    auto predecessors = relaxationManager->getAllPredecessors();
    auto distances = relaxationManager->getAllDistances();
    
    treeManager->buildTree(startNode, predecessors, distances);
}

std::vector<std::vector<int>> Dijkstra::findAllShortestPaths(int startId) {
    // One shortest path per reachable node, in node-id order; the start's own path is {startId}.
    std::vector<std::vector<int>> paths;
    if (!graph->hasNode(startId)) {
        return paths;
    }
    findShortestDistances(startId);
    buildShortestPathTree(startId);  // keep the tree queries in step with this source

    std::vector<int> nodeIds = graph->getAllNodeIds();
    std::sort(nodeIds.begin(), nodeIds.end());
    const auto predecessors = getAllPredecessors();
    for (int nodeId : nodeIds) {
        if (isNodeReachable(nodeId)) {
            std::vector<int> path = reconstructPath(predecessors, startId, nodeId);
            if (!path.empty()) {
                paths.push_back(std::move(path));
            }
        }
    }
    return paths;
}

std::vector<int> Dijkstra::getShortestPathInTree(int targetNode) const {
    if (!treeManager || !relaxationManager) {
        return {};
    }
    
    auto predecessors = relaxationManager->getAllPredecessors();
    return treeManager->getPathTo(targetNode, predecessors);
}

std::vector<int> Dijkstra::getTreeChildren(int nodeId) const {
    return treeManager ? treeManager->getChildren(nodeId) : std::vector<int>();
}

bool Dijkstra::isNodeInTree(int nodeId) const {
    return treeManager && treeManager->isInTree(nodeId);
}

size_t Dijkstra::getShortestPathTreeSize() const {
    return treeManager ? treeManager->getTreeSize() : 0;
}

void Dijkstra::printShortestPathTree() const {
    if (treeManager) {
        treeManager->printTreeStructure();
    } else {
        std::cout << "[DIJKSTRA] No shortest path tree available" << std::endl;
    }
}

void Dijkstra::clearShortestPathTree() {
    if (treeManager) {
        treeManager.reset();
    }
}