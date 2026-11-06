#include "environment/MapUpdater.hpp"
#include <iostream>
#include <stdexcept>
#include <string>
#include <chrono>

// MapUpdater edits the live graph and records each change. Sensor integration and database
// synchronisation live in sensor_data_integration.cpp and map_synchronization.cpp.

MapUpdate::MapUpdate(UpdateType t) : type(t), timestamp(std::chrono::steady_clock::now()) {}

MapUpdater::MapUpdater(Graph* graph)
    : environment(graph), realTimeUpdates(false), lastUpdateTime(std::chrono::steady_clock::now()) {
    if (!environment) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
}

void MapUpdater::processNodeAddition(const MapUpdate& update) {
    updateHistory.push_back(update);
    lastUpdateTime = update.timestamp;
}

void MapUpdater::processNodeRemoval(const MapUpdate& update) {
    updateHistory.push_back(update);
    lastUpdateTime = update.timestamp;
}

void MapUpdater::processEdgeModification(const MapUpdate& update) {
    updateHistory.push_back(update);
    lastUpdateTime = update.timestamp;
}

void MapUpdater::addNode(int nodeId, const std::string& name, double x, double y) {
    if (environment->hasNode(nodeId)) {
        throw std::invalid_argument("Node " + std::to_string(nodeId) + " already exists");
    }
    environment->addNode(nodeId, name, x, y);
    MapUpdate update(UpdateType::NODE_ADDITION);
    update.affectedNodes.push_back(nodeId);
    processNodeAddition(update);
}

void MapUpdater::removeNode(int nodeId) {
    if (!environment->removeNode(nodeId)) {
        throw std::invalid_argument("Node " + std::to_string(nodeId) + " does not exist");
    }
    MapUpdate update(UpdateType::NODE_REMOVAL);
    update.affectedNodes.push_back(nodeId);
    processNodeRemoval(update);
}

void MapUpdater::updateEdgeWeight(int fromId, int toId, double newWeight) {
    if (!environment->hasEdge(fromId, toId)) {
        throw std::invalid_argument("No edge " + std::to_string(fromId) + " -> " + std::to_string(toId));
    }
    // Graph has no in-place weight setter: replace the edge, keeping its direction setting.
    bool bidirectional = true;
    for (const Edge& edge : environment->getEdgesFrom(fromId)) {
        if (edge.getToNode() == toId) {
            bidirectional = edge.isBidirectional();
            break;
        }
    }
    environment->removeEdge(fromId, toId);
    if (bidirectional) {
        environment->removeEdge(toId, fromId);
    }
    environment->addEdge(fromId, toId, newWeight, bidirectional);

    MapUpdate update(UpdateType::WEIGHT_MODIFICATION);
    update.affectedEdges.emplace_back(fromId, toId);
    processEdgeModification(update);
}

void MapUpdater::integrateExternalMap(const Graph& externalMap) {
    MapUpdate update(UpdateType::FULL_RECONSTRUCTION);
    for (int nodeId : externalMap.getAllNodeIds()) {
        if (!environment->hasNode(nodeId)) {
            const Node& node = externalMap.getNode(nodeId);
            environment->addNode(nodeId, node.getName(), node.getX(), node.getY());
            update.affectedNodes.push_back(nodeId);
        }
    }
    for (int nodeId : externalMap.getAllNodeIds()) {
        for (const Edge& edge : externalMap.getEdgesFrom(nodeId)) {
            if (!environment->hasEdge(edge.getFromNode(), edge.getToNode())) {
                environment->addEdge(edge.getFromNode(), edge.getToNode(), edge.getWeight(), false);
                update.affectedEdges.emplace_back(edge.getFromNode(), edge.getToNode());
            }
        }
    }
    updateHistory.push_back(update);
    lastUpdateTime = update.timestamp;
}

std::vector<MapUpdate> MapUpdater::getRecentUpdates(std::chrono::duration<double> timeWindow) const {
    const auto cutoff = std::chrono::steady_clock::now() -
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(timeWindow);
    std::vector<MapUpdate> recent;
    for (const MapUpdate& update : updateHistory) {
        if (update.timestamp >= cutoff) {
            recent.push_back(update);
        }
    }
    return recent;
}

void MapUpdater::clearUpdateHistory() {
    updateHistory.clear();
}

void MapUpdater::enableRealTimeUpdates(bool enable) {
    realTimeUpdates = enable;
}

size_t MapUpdater::getUpdateCount() const {
    return updateHistory.size();
}

std::chrono::steady_clock::time_point MapUpdater::getLastUpdateTime() const {
    return lastUpdateTime;
}
