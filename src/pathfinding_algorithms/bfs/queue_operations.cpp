#include "pathfinding_algorithms/BFS.hpp"
#include "bfs_internals.hpp"
#include <queue>
#include <iostream>
#include <stdexcept>
#include <memory>
#include <utility>

void BFS::initializeQueue() {
    if (!queueManager) {
        queueManager = std::make_unique<QueueManager>();
    }
    queueManager->clear();
    queueManager->resetStatistics();
}

void BFS::enqueueNode(int nodeId) {
    if (!queueManager) {
        initializeQueue();
    }
    queueManager->enqueue(nodeId);
}

void BFS::enqueueNodeWithDistance(int nodeId, int distance) {
    if (!queueManager) {
        initializeQueue();
    }
    queueManager->enqueueWithDistance(nodeId, distance);
}

int BFS::dequeueNode() {
    if (!queueManager) {
        throw std::runtime_error("Queue not initialized");
    }
    return queueManager->dequeue();
}

std::pair<int, int> BFS::dequeueNodeWithDistance() {
    if (!queueManager) {
        throw std::runtime_error("Queue not initialized");
    }
    return queueManager->dequeueWithDistance();
}

bool BFS::isQueueEmpty() const {
    return !queueManager || queueManager->empty();
}

size_t BFS::getQueueSize() const {
    return queueManager ? queueManager->size() : 0;
}

size_t BFS::getMaxQueueSize() const {
    return queueManager ? queueManager->getMaxQueueSize() : 0;
}

void BFS::clearQueue() {
    if (queueManager) {
        queueManager->clear();
    }
}

void BFS::printQueueStatistics() const {
    if (queueManager) {
        queueManager->printStatistics();
    } else {
        std::cout << "[BFS] Queue manager not initialized" << std::endl;
    }
}