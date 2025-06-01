#include "../../../include/core/Edge.hpp"
#include <stdexcept>

Edge::Edge(int from, int to, double edgeWeight, bool isBidirectional)
    : fromNodeId(from), toNodeId(to), weight(edgeWeight), bidirectional(isBidirectional) {
    
    if (from == to) {
        throw std::invalid_argument("Cannot create edge from node to itself");
    }
    if (edgeWeight < 0) {
        throw std::invalid_argument("Edge weight cannot be negative");
    }
}

int Edge::getFromNode() const {
    return fromNodeId;
}

int Edge::getToNode() const {
    return toNodeId;
}

double Edge::getWeight() const {
    return weight;
}

bool Edge::isBidirectional() const {
    return bidirectional;
}