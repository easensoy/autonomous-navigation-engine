#include "../../../include/pathfinding_algorithms/BFS.hpp"
#include <queue>
#include <iostream>
#include <stdexcept>

class BFS::QueueManager {
private:
    std::queue<int> nodeQueue;
    std::queue<int> distanceQueue;
    size_t maxQueueSize;
    size_t totalEnqueued;
    size_t totalDequeued;
    
public:
    QueueManager() : maxQueueSize(0), totalEnqueued(0), totalDequeued(0) {}
    
    void enqueue(int nodeId) {
        nodeQueue.push(nodeId);
        totalEnqueued++;
        
        if (nodeQueue.size() > maxQueueSize) {
            maxQueueSize = nodeQueue.size();
        }
    }
    
    void enqueueWithDistance(int nodeId, int distance) {
        nodeQueue.push(nodeId);
        distanceQueue.push(distance);
        totalEnqueued++;
        
        if (nodeQueue.size() > maxQueueSize) {
            maxQueueSize = nodeQueue.size();
        }
    }
    
    int dequeue() {
        if (nodeQueue.empty()) {
            throw std::runtime_error("Cannot dequeue from empty queue");
        }
        
        int node = nodeQueue.front();
        nodeQueue.pop();
        totalDequeued++;
        
        if (!distanceQueue.empty()) {
            distanceQueue.pop();
        }
        
        return node;
    }
    
    std::pair<int, int> dequeueWithDistance() {
        if (nodeQueue.empty() || distanceQueue.empty()) {
            throw std::runtime_error("Cannot dequeue from empty queue");
        }
        
        int node = nodeQueue.front();
        int distance = distanceQueue.front();
        
        nodeQueue.pop();
        distanceQueue.pop();
        totalDequeued++;
        
        return {node, distance};
    }
    
    int front() const {
        if (nodeQueue.empty()) {
            throw std::runtime_error("Queue is empty");
        }
        return nodeQueue.front();
    }
    
    bool empty() const {
        return nodeQueue.empty();
    }
    
    size_t size() const {
        return nodeQueue.size();
    }
    
    void clear() {
        while (!nodeQueue.empty()) {
            nodeQueue.pop();
        }
        while (!distanceQueue.empty()) {
            distanceQueue.pop();
        }
    }
    
    size_t getMaxQueueSize() const {
        return maxQueueSize;
    }
    
    size_t getTotalEnqueued() const {
        return totalEnqueued;
    }
    
    size_t getTotalDequeued() const {
        return totalDequeued;
    }
    
    void resetStatistics() {
        maxQueueSize = nodeQueue.size();
        totalEnqueued = 0;
        totalDequeued = 0;
    }
    
    void printStatistics() const {
        std::cout << "[BFS_QUEUE] Queue Statistics:" << std::endl;
        std::cout << "[BFS_QUEUE]   Current size: " << size() << std::endl;
        std::cout << "[BFS_QUEUE]   Maximum size reached: " << maxQueueSize << std::endl;
        std::cout << "[BFS_QUEUE]   Total enqueued: " << totalEnqueued << std::endl;
        std::cout << "[BFS_QUEUE]   Total dequeued: " << totalDequeued << std::endl;
        std::cout << "[BFS_QUEUE]   Remaining in queue: " << (totalEnqueued - totalDequeued) << std::endl;
    }
};

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