#include "../../../include/core/Graph.hpp"
#include <iostream>
#include <stdexcept>

void Graph::addNode(int id, const std::string& name, double x, double y) {
    if (hasNode(id)) {
        throw std::invalid_argument("Node with ID " + std::to_string(id) + " already exists");
    }
    
    nodes[id] = std::make_unique<Node>(id, name, x, y);
    adjacencyList[id] = std::vector<Edge>();
    
    std::cout << "Added node: " << *nodes[id] << std::endl;
}

void Graph::addNode(const Node& node) {
    addNode(node.getId(), node.getName(), node.getX(), node.getY());
}

bool Graph::removeNode(int nodeId) {
    if (!hasNode(nodeId)) {
        return false;
    }
    
    for (auto& pair : adjacencyList) {
        auto& edges = pair.second;
        edges.erase(
            std::remove_if(edges.begin(), edges.end(),
                [nodeId](const Edge& edge) {
                    return edge.getToNode() == nodeId || edge.getFromNode() == nodeId;
                }),
            edges.end()
        );
    }
    
    nodes.erase(nodeId);
    adjacencyList.erase(nodeId);
    
    return true;
}

bool Graph::hasNode(int nodeId) const {
    return nodes.find(nodeId) != nodes.end();
}

const Node& Graph::getNode(int nodeId) const {
    validateNodeExists(nodeId);
    return *nodes.at(nodeId);
}

Node& Graph::getNode(int nodeId) {
    validateNodeExists(nodeId);
    return *nodes.at(nodeId);
}