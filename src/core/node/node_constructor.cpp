#include "core/Node.hpp"
#include <stdexcept>
#include <iostream>

Node::Node(int nodeId, const std::string& nodeName, double xPos, double yPos)
    : id(nodeId), name(nodeName), x(xPos), y(yPos) {
    
    if (nodeId < 0) {
        throw std::invalid_argument("Node ID must be non-negative. Received: " + std::to_string(nodeId));
    }
    
    if (nodeName.empty()) {
        throw std::invalid_argument("Node name cannot be empty");
    }
    
    static const double MAX_COORDINATE = 10000.0;
    static const double MIN_COORDINATE = -10000.0;
    
    if (xPos < MIN_COORDINATE || xPos > MAX_COORDINATE) {
        throw std::invalid_argument("X coordinate out of valid range");
    }
    
    if (yPos < MIN_COORDINATE || yPos > MAX_COORDINATE) {
        throw std::invalid_argument("Y coordinate out of valid range");
    }
}

Node::Node(const Node& other) 
    : id(other.id), name(other.name), x(other.x), y(other.y) {
}

Node& Node::operator=(const Node& other) {
    if (this == &other) {
        return *this;
    }
    
    id = other.id;
    name = other.name;
    x = other.x;
    y = other.y;
    
    return *this;
}

int Node::getId() const {
    return id;
}

const std::string& Node::getName() const {
    return name;
}

double Node::getX() const {
    return x;
}

double Node::getY() const {
    return y;
}