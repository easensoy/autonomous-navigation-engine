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
    ++revision;
    
    std::cout << "Added edge: " << fromId << " -> " << toId 
              << " (weight: " << weight << ")" << std::endl;
}

void Graph::addEdge(const Edge& edge) {
    addEdge(edge.getFromNode(), edge.getToNode(), edge.getWeight(), edge.isBidirectional());
}

// Removes the edge fromId -> toId. A two-way edge is stored once per direction, so removing it
// also removes the reverse direction; a one-way edge is removed on its own.
bool Graph::removeEdge(int fromId, int toId) {
    if (!hasNode(fromId) || !hasNode(toId)) {
        return false;
    }

    auto fromList = adjacencyList.find(fromId);
    if (fromList == adjacencyList.end()) {
        return false;
    }
    auto& fromEdges = fromList->second;
    auto it = std::find_if(fromEdges.begin(), fromEdges.end(),
        [toId](const Edge& edge) { return edge.getToNode() == toId; });
    if (it == fromEdges.end()) {
        return false;
    }

    const bool twoWay = it->isBidirectional();
    fromEdges.erase(it);
    ++revision;

    if (twoWay) {
        auto toList = adjacencyList.find(toId);
        if (toList != adjacencyList.end()) {
            auto& toEdges = toList->second;
            auto twin = std::find_if(toEdges.begin(), toEdges.end(), [fromId](const Edge& edge) {
                return edge.getToNode() == fromId && edge.isBidirectional();
            });
            if (twin != toEdges.end()) {
                toEdges.erase(twin);
            }
        }
    }
    return true;
}

bool Graph::hasEdge(int fromId, int toId) const {
    if (!hasNode(fromId) || !hasNode(toId)) {
        return false;
    }
    
    const auto& edges = getEdgesFrom(fromId);
    return std::any_of(edges.begin(), edges.end(),
        [toId](const Edge& edge) { return edge.getToNode() == toId; });
}