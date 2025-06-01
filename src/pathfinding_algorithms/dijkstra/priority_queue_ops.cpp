#include "../../../include/pathfinding_algorithms/Dijkstra.hpp"
#include <queue>
#include <unordered_map>
#include <iostream>

class Dijkstra::PriorityQueueManager {
private:
    std::priority_queue<DijkstraNode, std::vector<DijkstraNode>, std::greater<DijkstraNode>> pq;
    std::unordered_map<int, double> bestDistances;
    
public:
    void push(const DijkstraNode& node) {
        auto it = bestDistances.find(node.nodeId);
        if (it == bestDistances.end() || node.distance < it->second) {
            pq.push(node);
            bestDistances[node.nodeId] = node.distance;
        }
    }
    
    DijkstraNode pop() {
        while (!pq.empty()) {
            DijkstraNode current = pq.top();
            pq.pop();
            
            auto it = bestDistances.find(current.nodeId);
            if (it != bestDistances.end() && current.distance <= it->second) {
                return current;
            }
        }
        throw std::runtime_error("Priority queue is empty");
    }
    
    bool empty() const {
        return pq.empty();
    }
    
    size_t size() const {
        return pq.size();
    }
    
    void clear() {
        while (!pq.empty()) {
            pq.pop();
        }
        bestDistances.clear();
    }
    
    bool hasImprovedDistance(int nodeId, double distance) const {
        auto it = bestDistances.find(nodeId);
        return it == bestDistances.end() || distance < it->second;
    }
    
    void updateBestDistance(int nodeId, double distance) {
        bestDistances[nodeId] = distance;
    }
};

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