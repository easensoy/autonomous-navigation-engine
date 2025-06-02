#include "core/Edge.hpp"

std::ostream& operator<<(std::ostream& os, const Edge& edge) {
    os << "Edge[" << edge.fromNodeId;
    if (edge.bidirectional) {
        os << " <-> ";
    } else {
        os << " -> ";
    }
    os << edge.toNodeId << ", weight:" << edge.weight << "]";
    return os;
}