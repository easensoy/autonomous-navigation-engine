#include "../../../include/pathfinding_algorithms/BellmanFord.hpp"
#include <iostream>
#include <limits>
#include <unordered_set>

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

bool BellmanFord::initializeNegativeCycleDetection() {
    if (!cycleDetector) {
        cycleDetector = std::make_unique<NegativeCycleDetector>(graph);
        return true;
    }
    return false;
}

bool BellmanFord::hasNegativeCycleInDistances(const std::unordered_map<int, double>& distances,
                                             const std::unordered_map<int, int>& predecessors) {
    if (!cycleDetector) {
        initializeNegativeCycleDetection();
    }
    
    return cycleDetector->hasNegativeCycle(distances, predecessors);
}

std::vector<int> BellmanFord::getLastDetectedNegativeCycle() const {
    return cycleDetector ? cycleDetector->getLastDetectedCycle() : std::vector<int>();
}

std::vector<int> BellmanFord::findAllNegativeCycles() {
    if (!cycleDetector) {
        initializeNegativeCycleDetection();
    }
    
    return cycleDetector->findAllNegativeCycles();
}

bool BellmanFord::isNodeInNegativeCycle(int nodeId) {
    if (!cycleDetector) {
        initializeNegativeCycleDetection();
    }
    
    return cycleDetector->isNodeInNegativeCycle(nodeId);
}

void BellmanFord::enableNegativeCycleDetection(bool enabled) {
    if (!cycleDetector) {
        initializeNegativeCycleDetection();
    }
    
    cycleDetector->setDetectionEnabled(enabled);
}

double BellmanFord::calculateNegativeCycleWeight(const std::vector<int>& cycle) const {
    if (!cycleDetector) {
        return 0.0;
    }
    
    return cycleDetector->calculateCycleWeight(cycle);
}