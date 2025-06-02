#include "core/Edge.hpp"

bool Edge::operator==(const Edge& other) const {
    return fromNodeId == other.fromNodeId && 
           toNodeId == other.toNodeId && 
           weight == other.weight;
}

bool Edge::operator<(const Edge& other) const {
    return weight < other.weight;
}