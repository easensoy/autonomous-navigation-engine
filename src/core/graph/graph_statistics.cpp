#include "../../../include/core/Graph.hpp"

size_t Graph::getNodeCount() const {
    return nodes.size();
}

size_t Graph::getEdgeCount() const {
    size_t count = 0;
    for (const auto& pair : adjacencyList) {
        count += pair.second.size();
    }
    return count / 2;
}

void Graph::clear() {
    nodes.clear();
    adjacencyList.clear();
}

bool Graph::isEmpty() const {
    return nodes.empty();
}

std::vector<int> Graph::getAllNodeIds() const {
    std::vector<int> ids;
    for (const auto& pair : nodes) {
        ids.push_back(pair.first);
    }
    return ids;
}

double Graph::getEdgeWeight(int fromId, int toId) const {
    const auto& edges = getEdgesFrom(fromId);
    for (const auto& edge : edges) {
        if (edge.getToNode() == toId) {
            return edge.getWeight();
        }
    }
    return -1.0;
}

void Graph::validateNodeExists(int nodeId) const {
    if (!hasNode(nodeId)) {
        throw std::invalid_argument("Node with ID " + std::to_string(nodeId) + " does not exist");
    }
}

void Graph::printGraphStructure() const {
    std::cout << "\n=== GRAPH STRUCTURE ===" << std::endl;
    std::cout << "Nodes: " << getNodeCount() << ", Edges: " << getEdgeCount() << std::endl;
    
    for (const auto& pair : nodes) {
        const Node& node = *pair.second;
        std::cout << "\n" << node << std::endl;
        
        const auto& edges = getEdgesFrom(node.getId());
        if (!edges.empty()) {
            std::cout << "  Connections:" << std::endl;
            for (const auto& edge : edges) {
                const Node& neighbor = getNode(edge.getToNode());
                std::cout << "    -> " << neighbor.getName() 
                          << " (weight: " << edge.getWeight() << ")" << std::endl;
            }
        } else {
            std::cout << "  No connections." << std::endl;
        }
    }
    std::cout << "========================\n" << std::endl;
}