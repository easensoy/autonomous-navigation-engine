#include "graph_operations/NodeManager.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

// NodeManager adds, removes and queries nodes and hands out unused node ids.

NodeManager::NodeManager(Graph* environment) : graph(environment), nextNodeId(0) {
    if (!graph) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
    for (int id : graph->getAllNodeIds()) {
        nextNodeId = std::max(nextNodeId, id + 1);
    }
}

bool NodeManager::validateNodePlacement(double x, double y) const {
    if (!std::isfinite(x) || !std::isfinite(y)) {
        return false;
    }
    // Reject a node placed exactly on top of an existing one.
    for (int id : graph->getAllNodeIds()) {
        const Node& node = graph->getNode(id);
        if (std::abs(node.getX() - x) < 1e-9 && std::abs(node.getY() - y) < 1e-9) {
            return false;
        }
    }
    return true;
}

int NodeManager::generateUniqueNodeId() {
    while (graph->hasNode(nextNodeId)) {
        ++nextNodeId;
    }
    return nextNodeId++;
}

int NodeManager::addNode(const std::string& name, double x, double y) {
    if (!validateNodePlacement(x, y)) {
        throw std::invalid_argument("Invalid or occupied position for node '" + name + "'");
    }
    const int id = generateUniqueNodeId();
    graph->addNode(id, name, x, y);
    return id;
}

int NodeManager::addNodeWithId(int nodeId, const std::string& name, double x, double y) {
    if (graph->hasNode(nodeId) || !validateNodePlacement(x, y)) {
        return -1;
    }
    graph->addNode(nodeId, name, x, y);
    nextNodeId = std::max(nextNodeId, nodeId + 1);
    return nodeId;
}

bool NodeManager::removeNode(int nodeId) {
    return graph->removeNode(nodeId);
}

std::vector<int> NodeManager::findNodesInRadius(double centerX, double centerY, double radius) const {
    std::vector<int> found;
    for (int id : graph->getAllNodeIds()) {
        const Node& node = graph->getNode(id);
        if (std::hypot(node.getX() - centerX, node.getY() - centerY) <= radius) {
            found.push_back(id);
        }
    }
    std::sort(found.begin(), found.end());
    return found;
}

int NodeManager::findNearestNode(double x, double y) const {
    int nearest = -1;
    double best = std::numeric_limits<double>::infinity();
    for (int id : graph->getAllNodeIds()) {
        const Node& node = graph->getNode(id);
        const double d = std::hypot(node.getX() - x, node.getY() - y);
        if (d < best || (d == best && id < nearest)) {
            best = d;
            nearest = id;
        }
    }
    return nearest;
}

std::vector<int> NodeManager::getIsolatedNodes() const {
    std::vector<int> isolated;
    for (int id : graph->getAllNodeIds()) {
        bool hasIncoming = false;
        if (graph->getEdgesFrom(id).empty()) {
            for (int other : graph->getAllNodeIds()) {
                if (graph->hasEdge(other, id)) {
                    hasIncoming = true;
                    break;
                }
            }
            if (!hasIncoming) {
                isolated.push_back(id);
            }
        }
    }
    std::sort(isolated.begin(), isolated.end());
    return isolated;
}

std::vector<int> NodeManager::getHighDegreeNodes(int minDegree) const {
    std::vector<int> result;
    for (int id : graph->getAllNodeIds()) {
        if (static_cast<int>(graph->getEdgesFrom(id).size()) >= minDegree) {
            result.push_back(id);
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}

std::vector<int> NodeManager::clusterNodes(double maxDistance) const {
    // Returns a cluster label per node, in ascending node-id order (single-linkage on distance).
    std::vector<int> ids = getAllNodeIds();
    std::vector<int> label(ids.size(), -1);
    int next = 0;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (label[i] != -1) continue;
        label[i] = next;
        std::vector<size_t> stack{i};
        while (!stack.empty()) {
            const size_t current = stack.back();
            stack.pop_back();
            const Node& a = graph->getNode(ids[current]);
            for (size_t j = 0; j < ids.size(); ++j) {
                if (label[j] != -1) continue;
                const Node& b = graph->getNode(ids[j]);
                if (std::hypot(a.getX() - b.getX(), a.getY() - b.getY()) <= maxDistance) {
                    label[j] = next;
                    stack.push_back(j);
                }
            }
        }
        ++next;
    }
    return label;
}

size_t NodeManager::getNodeCount() const {
    return graph->getNodeCount();
}

std::vector<int> NodeManager::getAllNodeIds() const {
    std::vector<int> ids = graph->getAllNodeIds();
    std::sort(ids.begin(), ids.end());
    return ids;
}

bool NodeManager::validateNodeConsistency() const {
    for (int id : graph->getAllNodeIds()) {
        if (graph->getNode(id).getId() != id) {
            return false;
        }
        for (const Edge& edge : graph->getEdgesFrom(id)) {
            if (!graph->hasNode(edge.getToNode())) {
                return false;
            }
        }
    }
    return true;
}
