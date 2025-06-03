#include "core/Graph.hpp"
#include <iostream>
#include <algorithm>

bool Graph::removeEdge(int fromId, int toId) {
    std::cout << "[GRAPH] Attempting to remove edge: " << fromId << " -> " << toId << std::endl;
    
    // Validate that both nodes exist
    if (!hasNode(fromId)) {
        std::cout << "[GRAPH] Cannot remove edge - source node " << fromId << " does not exist" << std::endl;
        return false;
    }
    
    if (!hasNode(toId)) {
        std::cout << "[GRAPH] Cannot remove edge - target node " << toId << " does not exist" << std::endl;
        return false;
    }
    
    bool edgeRemoved = false;
    
    // Remove edge from fromId to toId
    auto& fromEdges = adjacencyList[fromId];
    auto it = std::find_if(fromEdges.begin(), fromEdges.end(),
        [toId](const Edge& edge) { return edge.getToNode() == toId; });
    
    if (it != fromEdges.end()) {
        bool wasBidirectional = it->isBidirectional();
        fromEdges.erase(it);
        edgeRemoved = true;
        
        std::cout << "[GRAPH] Removed edge " << fromId << " -> " << toId << std::endl;
        
        // If the edge was bidirectional, also remove the reverse edge
        if (wasBidirectional) {
            auto& toEdges = adjacencyList[toId];
            auto reverseIt = std::find_if(toEdges.begin(), toEdges.end(),
                [fromId](const Edge& edge) { return edge.getToNode() == fromId; });
            
            if (reverseIt != toEdges.end()) {
                toEdges.erase(reverseIt);
                std::cout << "[GRAPH] Removed bidirectional edge " << toId << " -> " << fromId << std::endl;
            }
        }
    } else {
        std::cout << "[GRAPH] Edge " << fromId << " -> " << toId << " not found" << std::endl;
    }
    
    return edgeRemoved;
}

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