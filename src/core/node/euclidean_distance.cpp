#include "../../../include/core/Node.hpp"
#include <cmath>

double Node::distanceTo(const Node& other) const {
    return euclideanDistance(other);
}

double Node::euclideanDistance(const Node& other) const {
    double deltaX = x - other.x;
    double deltaY = y - other.y;
    
    double distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    
    static const double COORDINATE_TO_METER_SCALE = 0.1;
    return distance * COORDINATE_TO_METER_SCALE;
}