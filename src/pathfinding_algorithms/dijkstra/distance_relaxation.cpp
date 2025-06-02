#include "pathfinding_algorithms/Dijkstra.hpp"
#include <iostream>
#include <limits>

class Dijkstra::DistanceRelaxationManager {
private:
    std::unordered_map<int, double> distances;
    std::unordered_map<int, int> predecessors;
    bool enableDetailedLogging;
    
public:
    DistanceRelaxationManager() : enableDetailedLogging(false) {}
    
    void initializeDistances(const std::vector<int>& nodeIds, int startId) {
        distances.clear();
        predecessors.clear();
        
        for (int nodeId : nodeIds) {
            distances[nodeId] = (nodeId == startId) ? 0.0 : std::numeric_limits<double>::infinity();
            predecessors[nodeId] = -1;
        }
        
        if (enableDetailedLogging) {
            std::cout << "[DIJKSTRA] Initialized distances for " << nodeIds.size() << " nodes" << std::endl;
            std::cout << "[DIJKSTRA] Start node " << startId << " distance set to 0" << std::endl;
        }
    }
    
    bool relaxEdge(int fromNode, int toNode, double edgeWeight) {
        if (distances.find(fromNode) == distances.end() || 
            distances.find(toNode) == distances.end()) {
            return false;
        }
        
        double currentFromDistance = distances[fromNode];
        double currentToDistance = distances[toNode];
        double newDistance = currentFromDistance + edgeWeight;
        
        if (newDistance < currentToDistance) {
            distances[toNode] = newDistance;
            predecessors[toNode] = fromNode;
            
            if (enableDetailedLogging) {
                std::cout << "[DIJKSTRA] Relaxed edge " << fromNode << " -> " << toNode 
                          << ": " << currentToDistance << " -> " << newDistance << std::endl;
            }
            
            return true;
        }
        
        return false;
    }
    
    double getDistance(int nodeId) const {
        auto it = distances.find(nodeId);
        return (it != distances.end()) ? it->second : std::numeric_limits<double>::infinity();
    }
    
    int getPredecessor(int nodeId) const {
        auto it = predecessors.find(nodeId);
        return (it != predecessors.end()) ? it->second : -1;
    }
    
    std::unordered_map<int, double> getAllDistances() const {
        return distances;
    }
    
    std::unordered_map<int, int> getAllPredecessors() const {
        return predecessors;
    }
    
    void setLogging(bool enabled) {
        enableDetailedLogging = enabled;
    }
    
    bool isReachable(int nodeId) const {
        auto it = distances.find(nodeId);
        return it != distances.end() && it->second != std::numeric_limits<double>::infinity();
    }
    
    size_t getReachableNodeCount() const {
        size_t count = 0;
        for (const auto& pair : distances) {
            if (pair.second != std::numeric_limits<double>::infinity()) {
                count++;
            }
        }
        return count;
    }
};

void Dijkstra::initializeDistanceRelaxation(const std::vector<int>& nodeIds, int startId) {
    if (!relaxationManager) {
        relaxationManager = std::make_unique<DistanceRelaxationManager>();
    }
    relaxationManager->initializeDistances(nodeIds, startId);
}

bool Dijkstra::relaxEdge(int fromNode, int toNode, double edgeWeight) {
    if (!relaxationManager) {
        throw std::runtime_error("Distance relaxation not initialized");
    }
    return relaxationManager->relaxEdge(fromNode, toNode, edgeWeight);
}

double Dijkstra::getCurrentDistance(int nodeId) const {
    if (!relaxationManager) {
        return std::numeric_limits<double>::infinity();
    }
    return relaxationManager->getDistance(nodeId);
}

int Dijkstra::getPredecessor(int nodeId) const {
    if (!relaxationManager) {
        return -1;
    }
    return relaxationManager->getPredecessor(nodeId);
}

std::unordered_map<int, double> Dijkstra::getAllCurrentDistances() const {
    if (!relaxationManager) {
        return {};
    }
    return relaxationManager->getAllDistances();
}

std::unordered_map<int, int> Dijkstra::getAllPredecessors() const {
    if (!relaxationManager) {
        return {};
    }
    return relaxationManager->getAllPredecessors();
}

bool Dijkstra::isNodeReachable(int nodeId) const {
    return relaxationManager && relaxationManager->isReachable(nodeId);
}

size_t Dijkstra::getReachableNodeCount() const {
    return relaxationManager ? relaxationManager->getReachableNodeCount() : 0;
}

void Dijkstra::setDistanceLogging(bool enabled) {
    if (relaxationManager) {
        relaxationManager->setLogging(enabled);
    }
}