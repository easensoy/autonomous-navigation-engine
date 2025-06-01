#include "../../../include/core/Edge.hpp"
#include <stdexcept>

bool Edge::connectsNodes(int nodeA, int nodeB) const {
    if (bidirectional) {
        return (fromNodeId == nodeA && toNodeId == nodeB) || 
               (fromNodeId == nodeB && toNodeId == nodeA);
    } else {
        return fromNodeId == nodeA && toNodeId == nodeB;
    }
}

int Edge::getOtherNode(int nodeId) const {
    if (nodeId == fromNodeId) {
        return toNodeId;
    } else if (nodeId == toNodeId && bidirectional) {
        return fromNodeId;
    } else {
        throw std::invalid_argument("Node is not connected by this edge");
    }
}