#include "core/Graph.hpp"
#include <queue>
#include <unordered_set>

const std::vector<Edge>& Graph::getEdgesFrom(int nodeId) const {
    static const std::vector<Edge> emptyVector;
    
    auto it = adjacencyList.find(nodeId);
    return (it != adjacencyList.end()) ? it->second : emptyVector;
}

std::vector<int> Graph::getNeighbors(int nodeId) const {
    std::vector<int> neighbors;
    const auto& edges = getEdgesFrom(nodeId);
    
    for (const auto& edge : edges) {
        neighbors.push_back(edge.getToNode());
    }
    
    return neighbors;
}

bool Graph::isConnected() const {
    if (nodes.empty()) {
        return true;
    }
    
    std::unordered_set<int> visited;
    std::queue<int> toVisit;
    
    int startNode = nodes.begin()->first;
    toVisit.push(startNode);
    visited.insert(startNode);
    
    while (!toVisit.empty()) {
        int current = toVisit.front();
        toVisit.pop();
        
        for (int neighbor : getNeighbors(current)) {
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                toVisit.push(neighbor);
            }
        }
    }
    
    return visited.size() == nodes.size();
}