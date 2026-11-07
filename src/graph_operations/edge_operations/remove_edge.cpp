#include "core/Graph.hpp"
#include <iostream>
#include <utility>

// Graph::removeEdge itself is defined in core/graph/edge_insertion.cpp; these helpers build on it.

// Additional helper function for batch edge removal
bool removeMultipleEdges(Graph* graph, const std::vector<std::pair<int, int>>& edgesToRemove) {
    if (!graph) {
        std::cout << "[GRAPH] Cannot remove edges - null graph pointer" << std::endl;
        return false;
    }
    
    std::cout << "[GRAPH] Removing batch of " << edgesToRemove.size() << " edges" << std::endl;
    
    size_t successfulRemovals = 0;
    for (const auto& [fromId, toId] : edgesToRemove) {
        if (graph->removeEdge(fromId, toId)) {
            successfulRemovals++;
        }
    }
    
    std::cout << "[GRAPH] Successfully removed " << successfulRemovals 
              << " out of " << edgesToRemove.size() << " edges" << std::endl;
    
    return successfulRemovals == edgesToRemove.size();
}

// Function to remove all edges connected to a specific node
size_t removeAllEdgesFromNode(Graph* graph, int nodeId) {
    if (!graph || !graph->hasNode(nodeId)) {
        std::cout << "[GRAPH] Cannot remove edges - invalid node " << nodeId << std::endl;
        return 0;
    }
    
    std::cout << "[GRAPH] Removing all edges from node " << nodeId << std::endl;
    
    size_t edgesRemoved = 0;
    
    // Get all neighbors before removal to avoid iterator invalidation
    std::vector<int> neighbors = graph->getNeighbors(nodeId);
    
    for (int neighbor : neighbors) {
        if (graph->removeEdge(nodeId, neighbor)) {
            edgesRemoved++;
        }
    }
    
    // Also check for incoming edges (for directed graphs)
    std::vector<int> allNodes = graph->getAllNodeIds();
    for (int otherNode : allNodes) {
        if (otherNode != nodeId && graph->hasEdge(otherNode, nodeId)) {
            if (graph->removeEdge(otherNode, nodeId)) {
                edgesRemoved++;
            }
        }
    }
    
    std::cout << "[GRAPH] Removed " << edgesRemoved << " edges from node " << nodeId << std::endl;
    
    return edgesRemoved;
}

// Function to remove edges based on weight threshold
size_t removeEdgesByWeight(Graph* graph, double minWeight, double maxWeight) {
    if (!graph) {
        std::cout << "[GRAPH] Cannot remove edges - null graph pointer" << std::endl;
        return 0;
    }
    
    std::cout << "[GRAPH] Removing edges with weight between " << minWeight 
              << " and " << maxWeight << std::endl;
    
    std::vector<std::pair<int, int>> edgesToRemove;
    std::vector<int> nodeIds = graph->getAllNodeIds();
    
    // Collect edges that meet the weight criteria
    for (int nodeId : nodeIds) {
        const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
        for (const Edge& edge : edges) {
            double weight = edge.getWeight();
            if (weight >= minWeight && weight <= maxWeight) {
                edgesToRemove.emplace_back(edge.getFromNode(), edge.getToNode());
            }
        }
    }
    
    // Remove the collected edges
    size_t removedCount = 0;
    for (const auto& [fromId, toId] : edgesToRemove) {
        if (graph->removeEdge(fromId, toId)) {
            removedCount++;
        }
    }
    
    std::cout << "[GRAPH] Removed " << removedCount << " edges based on weight criteria" << std::endl;
    
    return removedCount;
}