#include "core/Node.hpp"
#include <cmath>

double Node::manhattanDistance(const Node& other) const {
    double deltaX = std::abs(x - other.x);
    double deltaY = std::abs(y - other.y);
    
    double distance = deltaX + deltaY;
    
    static const double COORDINATE_TO_METER_SCALE = 0.1;
    return distance * COORDINATE_TO_METER_SCALE;
}