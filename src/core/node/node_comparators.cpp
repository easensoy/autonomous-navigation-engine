#include "core/Node.hpp"

bool Node::operator==(const Node& other) const {
    return id == other.id;
}

bool Node::operator!=(const Node& other) const {
    return !(*this == other);
}

bool Node::operator<(const Node& other) const {
    return id < other.id;
}

std::ostream& operator<<(std::ostream& os, const Node& node) {
    os << "Node[ID:" << node.id << ", Name:'" << node.name 
       << "', Pos:(" << node.x << "," << node.y << ")]";
    return os;
}