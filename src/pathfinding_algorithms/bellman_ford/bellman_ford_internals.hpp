#pragma once
// Private to the BellmanFord implementation (not installed). Defines the nested helper classes that
// BellmanFord.hpp only forward-declares, so every BellmanFord source file sees them complete and
// BellmanFord's out-of-line destructor can delete them.
#include "pathfinding_algorithms/BellmanFord.hpp"
#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <iomanip>
#include <utility>

// Moved verbatim from edge_relaxation.cpp.
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

// Moved verbatim from negative_cycle_detection.cpp.
class BellmanFord::NegativeCycleDetector {
private:
    const Graph* graph;
    bool detectionEnabled;
    std::vector<int> lastDetectedCycle;
    
public:
    explicit NegativeCycleDetector(const Graph* environment) 
        : graph(environment), detectionEnabled(true) {}
    
    bool hasNegativeCycle(const std::unordered_map<int, double>& distances,
                         const std::unordered_map<int, int>& predecessors) {
        if (!detectionEnabled) {
            return false;
        }
        
        std::cout << "[CYCLE_DETECTOR] Performing negative cycle detection..." << std::endl;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        // Additional relaxation pass to detect negative cycles
        for (int nodeId : nodeIds) {
            auto distIt = distances.find(nodeId);
            if (distIt == distances.end() || 
                distIt->second == std::numeric_limits<double>::infinity()) {
                continue; // Skip unreachable nodes
            }
            
            const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
            for (const Edge& edge : edges) {
                int neighborId = edge.getToNode();
                auto neighborDistIt = distances.find(neighborId);
                
                if (neighborDistIt != distances.end()) {
                    double newDistance = distIt->second + edge.getWeight();
                    
                    if (newDistance < neighborDistIt->second) {
                        std::cout << "[CYCLE_DETECTOR] Negative cycle detected via edge " 
                                  << nodeId << " -> " << neighborId << std::endl;
                        
                        lastDetectedCycle = traceCycle(neighborId, predecessors);
                        return true;
                    }
                }
            }
        }
        
        std::cout << "[CYCLE_DETECTOR] No negative cycles detected" << std::endl;
        return false;
    }
    
    std::vector<int> getLastDetectedCycle() const {
        return lastDetectedCycle;
    }
    
    std::vector<int> findAllNegativeCycles() {
        std::cout << "[CYCLE_DETECTOR] Searching for all negative cycles..." << std::endl;
        
        std::vector<int> allCycleNodes;
        std::unordered_set<int> processedNodes;
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        for (int startNode : nodeIds) {
            if (processedNodes.find(startNode) != processedNodes.end()) {
                continue;
            }
            
            auto result = runBellmanFordFrom(startNode);
            if (result.hasNegativeCycle) {
                for (int node : result.negativeCycle) {
                    allCycleNodes.push_back(node);
                    processedNodes.insert(node);
                }
            }
        }
        
        std::cout << "[CYCLE_DETECTOR] Found " << allCycleNodes.size() 
                  << " nodes involved in negative cycles" << std::endl;
        
        return allCycleNodes;
    }
    
    bool isNodeInNegativeCycle(int nodeId) {
        auto result = runBellmanFordFrom(nodeId);
        if (!result.hasNegativeCycle) {
            return false;
        }
        
        auto it = std::find(result.negativeCycle.begin(), result.negativeCycle.end(), nodeId);
        return it != result.negativeCycle.end();
    }
    
    void setDetectionEnabled(bool enabled) {
        detectionEnabled = enabled;
        std::cout << "[CYCLE_DETECTOR] Negative cycle detection " 
                  << (enabled ? "enabled" : "disabled") << std::endl;
    }
    
    double calculateCycleWeight(const std::vector<int>& cycle) const {
        if (cycle.size() < 2) {
            return 0.0;
        }
        
        double totalWeight = 0.0;
        for (size_t i = 0; i < cycle.size() - 1; ++i) {
            int fromNode = cycle[i];
            int toNode = cycle[i + 1];
            
            const std::vector<Edge>& edges = graph->getEdgesFrom(fromNode);
            bool edgeFound = false;
            
            for (const Edge& edge : edges) {
                if (edge.getToNode() == toNode) {
                    totalWeight += edge.getWeight();
                    edgeFound = true;
                    break;
                }
            }
            
            if (!edgeFound) {
                std::cout << "[CYCLE_DETECTOR] Warning: Missing edge in cycle between " 
                          << fromNode << " and " << toNode << std::endl;
                return std::numeric_limits<double>::infinity();
            }
        }
        
        return totalWeight;
    }
    
private:
    std::vector<int> traceCycle(int startNode, const std::unordered_map<int, int>& predecessors) {
        std::cout << "[CYCLE_DETECTOR] Tracing cycle starting from node " << startNode << std::endl;
        
        std::vector<int> cycle;
        std::unordered_set<int> visited;
        int current = startNode;
        
        // First, move backward to get into the cycle
        std::vector<int> nodeIds = graph->getAllNodeIds();
        for (size_t i = 0; i < nodeIds.size(); ++i) {
            auto it = predecessors.find(current);
            if (it != predecessors.end()) {
                current = it->second;
            } else {
                break;
            }
        }
        
        // Now trace the actual cycle
        int cycleStart = current;
        do {
            if (visited.find(current) != visited.end()) {
                break; // Found the cycle
            }
            
            visited.insert(current);
            cycle.push_back(current);
            
            auto it = predecessors.find(current);
            if (it != predecessors.end()) {
                current = it->second;
            } else {
                break;
            }
        } while (cycle.size() < nodeIds.size());
        
        // Find where the cycle actually starts
        auto cycleStartIt = std::find(cycle.begin(), cycle.end(), current);
        if (cycleStartIt != cycle.end()) {
            cycle.erase(cycle.begin(), cycleStartIt);
            cycle.push_back(current); // Complete the cycle
        }
        
        std::cout << "[CYCLE_DETECTOR] Traced cycle with " << cycle.size() << " nodes" << std::endl;
        return cycle;
    }
    
    struct CycleDetectionResult {
        bool hasNegativeCycle;
        std::vector<int> negativeCycle;
        std::unordered_map<int, double> distances;
        std::unordered_map<int, int> predecessors;
    };
    
    CycleDetectionResult runBellmanFordFrom(int startNode) {
        CycleDetectionResult result;
        result.hasNegativeCycle = false;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        // Initialize
        for (int nodeId : nodeIds) {
            result.distances[nodeId] = (nodeId == startNode) ? 0.0 : std::numeric_limits<double>::infinity();
            result.predecessors[nodeId] = -1;
        }
        
        // Relax edges
        for (size_t iteration = 0; iteration < nodeIds.size() - 1; ++iteration) {
            for (int nodeId : nodeIds) {
                if (result.distances[nodeId] == std::numeric_limits<double>::infinity()) {
                    continue;
                }
                
                const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
                for (const Edge& edge : edges) {
                    int neighborId = edge.getToNode();
                    double newDistance = result.distances[nodeId] + edge.getWeight();
                    
                    if (newDistance < result.distances[neighborId]) {
                        result.distances[neighborId] = newDistance;
                        result.predecessors[neighborId] = nodeId;
                    }
                }
            }
        }
        
        // Check for negative cycle
        result.hasNegativeCycle = hasNegativeCycle(result.distances, result.predecessors);
        if (result.hasNegativeCycle) {
            result.negativeCycle = lastDetectedCycle;
        }
        
        return result;
    }
};
