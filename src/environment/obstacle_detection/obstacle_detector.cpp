#include "environment/ObstacleDetector.hpp"
#include <algorithm>
#include <stdexcept>
#include <string>
#include <chrono>
#include <iterator>

// ObstacleDetector keeps the set of blocked nodes. A static obstacle always blocks its node; a
// dynamic one blocks it when its severity reaches the detection sensitivity.

Obstacle::Obstacle(int id, ObstacleType t, double s)
    : nodeId(id), type(t), severity(s), detectionTime(std::chrono::steady_clock::now()) {}

ObstacleDetector::ObstacleDetector(const Graph* graph) : environment(graph), detectionSensitivity(0.5) {
    if (!environment) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
}

bool ObstacleDetector::isNodeAccessible(int nodeId) const {
    return environment->hasNode(nodeId) && blockedNodes.count(nodeId) == 0;
}

void ObstacleDetector::updateObstacleStatus() {
    // Drop obstacles whose node no longer exists in the graph.
    detectedObstacles.erase(std::remove_if(detectedObstacles.begin(), detectedObstacles.end(),
                                           [this](const Obstacle& o) { return !environment->hasNode(o.nodeId); }),
                            detectedObstacles.end());
    for (auto it = blockedNodes.begin(); it != blockedNodes.end();) {
        it = environment->hasNode(*it) ? std::next(it) : blockedNodes.erase(it);
    }
}

std::vector<int> ObstacleDetector::detectObstacles() {
    updateObstacleStatus();
    return getBlockedNodes();
}

std::vector<int> ObstacleDetector::detectObstaclesInRegion(const std::vector<int>& regionNodes) {
    updateObstacleStatus();
    std::vector<int> found;
    for (int nodeId : regionNodes) {
        if (blockedNodes.count(nodeId)) {
            found.push_back(nodeId);
        }
    }
    return found;
}

void ObstacleDetector::addStaticObstacle(int nodeId) {
    if (!environment->hasNode(nodeId)) {
        throw std::invalid_argument("Node " + std::to_string(nodeId) + " does not exist");
    }
    detectedObstacles.emplace_back(nodeId, ObstacleType::STATIC, 1.0);
    blockedNodes.insert(nodeId);
}

void ObstacleDetector::addDynamicObstacle(int nodeId, double severity) {
    if (!environment->hasNode(nodeId)) {
        throw std::invalid_argument("Node " + std::to_string(nodeId) + " does not exist");
    }
    detectedObstacles.emplace_back(nodeId, ObstacleType::DYNAMIC, severity);
    if (severity >= detectionSensitivity) {
        blockedNodes.insert(nodeId);
    }
}

void ObstacleDetector::removeObstacle(int nodeId) {
    detectedObstacles.erase(std::remove_if(detectedObstacles.begin(), detectedObstacles.end(),
                                           [nodeId](const Obstacle& o) { return o.nodeId == nodeId; }),
                            detectedObstacles.end());
    blockedNodes.erase(nodeId);
}

void ObstacleDetector::clearAllObstacles() {
    detectedObstacles.clear();
    blockedNodes.clear();
}

bool ObstacleDetector::isNodeBlocked(int nodeId) const {
    return blockedNodes.count(nodeId) > 0;
}

std::vector<int> ObstacleDetector::getBlockedNodes() const {
    std::vector<int> nodes(blockedNodes.begin(), blockedNodes.end());
    std::sort(nodes.begin(), nodes.end());
    return nodes;
}

std::vector<Obstacle> ObstacleDetector::getAllObstacles() const {
    return detectedObstacles;
}

std::vector<int> ObstacleDetector::getObstaclesNearPath(const std::vector<int>& path) const {
    std::vector<int> near;
    auto addIfBlocked = [&](int nodeId) {
        if (blockedNodes.count(nodeId) && std::find(near.begin(), near.end(), nodeId) == near.end()) {
            near.push_back(nodeId);
        }
    };
    for (int nodeId : path) {
        addIfBlocked(nodeId);
        for (int neighbor : environment->getNeighbors(nodeId)) {
            addIfBlocked(neighbor);
        }
    }
    return near;
}

void ObstacleDetector::setDetectionSensitivity(double sensitivity) {
    detectionSensitivity = std::clamp(sensitivity, 0.0, 1.0);
}

void ObstacleDetector::updateObstacleMap(const std::vector<int>& sensorData) {
    // Sensor readings are node ids reported as occupied.
    for (int nodeId : sensorData) {
        if (environment->hasNode(nodeId) && !blockedNodes.count(nodeId)) {
            addDynamicObstacle(nodeId, 1.0);
        }
    }
}

double ObstacleDetector::getObstacleDensity() const {
    const size_t nodeCount = environment->getNodeCount();
    return nodeCount == 0 ? 0.0 : static_cast<double>(blockedNodes.size()) / static_cast<double>(nodeCount);
}
