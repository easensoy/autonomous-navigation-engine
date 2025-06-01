#include "../../../include/pathfinding_algorithms/BellmanFord.hpp"
#include <iostream>
#include <limits>

class BellmanFord::EdgeRelaxationManager {
private:
    std::unordered_map<int, double> distances;
    std::unordered_map<int, int> predecessors;
    size_t relaxationCount;
    size_t totalEdgeCount;
    bool detailedLogging;
    
public:
    EdgeRelaxationManager() : relaxationCount(0), totalEdgeCount(0), detailedLogging(false) {}
    
    void initializeDistances(const std::vector<int>& nodeIds, int startNode) {
        distances.clear();
        predecessors.clear();
        relaxationCount = 0;
        
        for (int nodeId : nodeIds) {
            distances[nodeId] = (nodeId == startNode) ? 0.0 : std::numeric_limits<double>::infinity();
            predecessors[nodeId] = -1;
        }
        
        if (detailedLogging) {
            std::cout << "[EDGE_RELAXATION] Initialized " << nodeIds.size() 
                      << " nodes with start node " << startNode << std::endl;
        }
    }
    
    bool relaxEdge(int fromNode, int toNode, double edgeWeight) {
        totalEdgeCount++;
        
        auto fromIt = distances.find(fromNode);
        auto toIt = distances.find(toNode);
        
        if (fromIt == distances.end() || toIt == distances.end()) {
            if (detailedLogging) {
                std::cout << "[EDGE_RELAXATION] Skipping edge " << fromNode 
                          << " -> " << toNode << " (nodes not in distance map)" << std::endl;
            }
            return false;
        }
        
        if (fromIt->second == std::numeric_limits<double>::infinity()) {
            if (detailedLogging) {
                std::cout << "[EDGE_RELAXATION] Skipping edge " << fromNode 
                          << " -> " << toNode << " (source unreachable)" << std::endl;
            }
            return false;
        }
        
        double currentDistance = toIt->second;
        double newDistance = fromIt->second + edgeWeight;
        
        if (newDistance < currentDistance) {
            distances[toNode] = newDistance;
            predecessors[toNode] = fromNode;
            relaxationCount++;
            
            if (detailedLogging) {
                std::cout << "[EDGE_RELAXATION] Relaxed edge " << fromNode 
                          << " -> " << toNode << ": " << currentDistance 
                          << " -> " << newDistance << " (improvement: " 
                          << (currentDistance - newDistance) << ")" << std::endl;
            }
            
            return true;
        }
        
        if (detailedLogging && newDistance == currentDistance) {
            std::cout << "[EDGE_RELAXATION] Edge " << fromNode << " -> " << toNode 
                      << " provides equal path cost " << newDistance << std::endl;
        }
        
        return false;
    }
    
    size_t performFullRelaxationPass(const Graph* graph) {
        size_t improvementsInPass = 0;
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        if (detailedLogging) {
            std::cout << "[EDGE_RELAXATION] Starting full relaxation pass over " 
                      << nodeIds.size() << " nodes" << std::endl;
        }
        
        for (int nodeId : nodeIds) {
            if (distances[nodeId] == std::numeric_limits<double>::infinity()) {
                continue; // Skip unreachable nodes
            }
            
            const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
            for (const Edge& edge : edges) {
                if (relaxEdge(nodeId, edge.getToNode(), edge.getWeight())) {
                    improvementsInPass++;
                }
            }
        }
        
        if (detailedLogging) {
            std::cout << "[EDGE_RELAXATION] Relaxation pass completed with " 
                      << improvementsInPass << " improvements" << std::endl;
        }
        
        return improvementsInPass;
    }
    
    std::unordered_map<int, double> getDistances() const {
        return distances;
    }
    
    std::unordered_map<int, int> getPredecessors() const {
        return predecessors;
    }
    
    size_t getRelaxationCount() const {
        return relaxationCount;
    }
    
    size_t getTotalEdgeCount() const {
        return totalEdgeCount;
    }
    
    double getRelaxationRatio() const {
        return totalEdgeCount > 0 ? static_cast<double>(relaxationCount) / totalEdgeCount : 0.0;
    }
    
    void setDetailedLogging(bool enabled) {
        detailedLogging = enabled;
    }
    
    void resetStatistics() {
        relaxationCount = 0;
        totalEdgeCount = 0;
    }
    
    std::vector<std::pair<int, double>> getReachableNodes() const {
        std::vector<std::pair<int, double>> reachable;
        
        for (const auto& pair : distances) {
            if (pair.second != std::numeric_limits<double>::infinity()) {
                reachable.emplace_back(pair.first, pair.second);
            }
        }
        
        std::sort(reachable.begin(), reachable.end(), 
                 [](const auto& a, const auto& b) { return a.second < b.second; });
        
        return reachable;
    }
    
    bool hasImprovedSince(int nodeId, double threshold) const {
        auto it = distances.find(nodeId);
        return it != distances.end() && it->second < threshold;
    }
    
    void printRelaxationStatistics() const {
        std::cout << "[EDGE_RELAXATION] Statistics:" << std::endl;
        std::cout << "[EDGE_RELAXATION]   Total edges processed: " << totalEdgeCount << std::endl;
        std::cout << "[EDGE_RELAXATION]   Successful relaxations: " << relaxationCount << std::endl;
        std::cout << "[EDGE_RELAXATION]   Relaxation ratio: " << std::fixed 
                  << std::setprecision(2) << (getRelaxationRatio() * 100) << "%" << std::endl;
        
        auto reachableNodes = getReachableNodes();
        std::cout << "[EDGE_RELAXATION]   Reachable nodes: " << reachableNodes.size() << std::endl;
    }
};

void BellmanFord::initializeEdgeRelaxation(const std::vector<int>& nodeIds, int startNode) {
    if (!relaxationManager) {
        relaxationManager = std::make_unique<EdgeRelaxationManager>();
    }
    
    relaxationManager->initializeDistances(nodeIds, startNode);
}

bool BellmanFord::relaxSingleEdge(int fromNode, int toNode, double edgeWeight) {
    if (!relaxationManager) {
        throw std::runtime_error("Edge relaxation not initialized");
    }
    
    return relaxationManager->relaxEdge(fromNode, toNode, edgeWeight);
}

size_t BellmanFord::performFullEdgeRelaxationPass() {
    if (!relaxationManager) {
        throw std::runtime_error("Edge relaxation not initialized");
    }
    
    return relaxationManager->performFullRelaxationPass(graph);
}

std::unordered_map<int, double> BellmanFord::getCurrentDistancesFromRelaxation() const {
    return relaxationManager ? relaxationManager->getDistances() : std::unordered_map<int, double>();
}

std::unordered_map<int, int> BellmanFord::getCurrentPredecessorsFromRelaxation() const {
    return relaxationManager ? relaxationManager->getPredecessors() : std::unordered_map<int, int>();
}

size_t BellmanFord::getEdgeRelaxationCount() const {
    return relaxationManager ? relaxationManager->getRelaxationCount() : 0;
}

size_t BellmanFord::getTotalEdgesProcessed() const {
    return relaxationManager ? relaxationManager->getTotalEdgeCount() : 0;
}

double BellmanFord::getEdgeRelaxationRatio() const {
    return relaxationManager ? relaxationManager->getRelaxationRatio() : 0.0;
}

void BellmanFord::setEdgeRelaxationLogging(bool enabled) {
    if (!relaxationManager) {
        relaxationManager = std::make_unique<EdgeRelaxationManager>();
    }
    
    relaxationManager->setDetailedLogging(enabled);
}

void BellmanFord::resetEdgeRelaxationStatistics() {
    if (relaxationManager) {
        relaxationManager->resetStatistics();
    }
}

std::vector<std::pair<int, double>> BellmanFord::getReachableNodesWithDistances() const {
    return relaxationManager ? relaxationManager->getReachableNodes() : 
           std::vector<std::pair<int, double>>();
}

void BellmanFord::printEdgeRelaxationStatistics() const {
    if (relaxationManager) {
        relaxationManager->printRelaxationStatistics();
    } else {
        std::cout << "[BELLMAN-FORD] Edge relaxation manager not initialized" << std::endl;
    }
}