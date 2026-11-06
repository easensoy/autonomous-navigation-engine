#include "pathfinding_algorithms/Dijkstra.hpp"
#include "dijkstra_internals.hpp"
#include <queue>
#include <unordered_map>
#include <iostream>
#include <memory>
#include <stdexcept>

void Dijkstra::initializePriorityQueue() {
    if (!pqManager) {
        pqManager = std::make_unique<PriorityQueueManager>();
    }
    pqManager->clear();
}

void Dijkstra::addToPriorityQueue(const DijkstraNode& node) {
    if (!pqManager) {
        initializePriorityQueue();
    }
    pqManager->push(node);
}

Dijkstra::DijkstraNode Dijkstra::getNextFromPriorityQueue() {
    if (!pqManager || pqManager->empty()) {
        throw std::runtime_error("Priority queue is empty");
    }
    return pqManager->pop();
}

bool Dijkstra::isPriorityQueueEmpty() const {
    return !pqManager || pqManager->empty();
}

size_t Dijkstra::getPriorityQueueSize() const {
    return pqManager ? pqManager->size() : 0;
}

void Dijkstra::clearPriorityQueue() {
    if (pqManager) {
        pqManager->clear();
    }
}

bool Dijkstra::shouldUpdateDistance(int nodeId, double newDistance) const {
    return pqManager && pqManager->hasImprovedDistance(nodeId, newDistance);
}