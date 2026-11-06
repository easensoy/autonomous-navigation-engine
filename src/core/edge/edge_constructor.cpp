#include "core/Edge.hpp"
#include <cmath>
#include <stdexcept>

Edge::Edge(int from, int to, double edgeWeight, bool isBidirectional)
    : fromNodeId(from), toNodeId(to), weight(edgeWeight), bidirectional(isBidirectional) {
    
    if (from == to) {
        throw std::invalid_argument("Cannot create edge from node to itself");
    }
    if (!std::isfinite(edgeWeight)) {
        throw std::invalid_argument("Edge weight must be a finite number");
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