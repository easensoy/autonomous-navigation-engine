#include "core/Graph.hpp"
#include <iostream>
#include <stdexcept>
#include <algorithm>

void Graph::addEdge(int fromId, int toId, double weight, bool bidirectional) {
    validateNodeExists(fromId);
    validateNodeExists(toId);
    
    if (fromId == toId) {
        throw std::invalid_argument("Cannot create edge from node to itself");
    }
    
    adjacencyList[fromId].emplace_back(fromId, toId, weight, bidirectional);
    
    if (bidirectional) {
        adjacencyList[toId].emplace_back(toId, fromId, weight, bidirectional);
    }
    
    std::cout << "Added edge: " << fromId << " -> " << toId 
              << " (weight: " << weight << ")" << std::endl;
}

void Graph::addEdge(const Edge& edge) {
    addEdge(edge.getFromNode(), edge.getToNode(), edge.getWeight(), edge.isBidirectional());
}

bool Graph::removeEdge(int fromId, int toId) {
    if (!hasNode(fromId) || !hasNode(toId)) {
        return false;
    }
    
    auto& fromEdges = adjacencyList[fromId];
    auto it = std::find_if(fromEdges.begin(), fromEdges.end(),
        [toId](const Edge& edge) { return edge.getToNode() == toId; });
    
    if (it != fromEdges.end()) {
        fromEdges.erase(it);
        return true;
    }
    
    return false;
}

bool Graph::hasEdge(int fromId, int toId) const {
    if (!hasNode(fromId) || !hasNode(toId)) {
        return false;
    }
    
    const auto& edges = getEdgesFrom(fromId);
    return std::any_of(edges.begin(), edges.end(),
        [toId](const Edge& edge) { return edge.getToNode() == toId; });
}