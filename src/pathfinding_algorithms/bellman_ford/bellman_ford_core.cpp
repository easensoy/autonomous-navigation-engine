#include "pathfinding_algorithms/BellmanFord.hpp"
#include <iostream>
#include <limits>
#include <algorithm>
#include <unordered_set>

BellmanFord::BellmanFord(const Graph* environment) : graph(environment) {
    if (!graph) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
}

std::vector<int> BellmanFord::findShortestPath(int startId, int goalId) {
    std::cout << "[BELLMAN-FORD] Finding shortest path from " << startId << " to " << goalId << std::endl;
    
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        throw std::invalid_argument("Start or goal node does not exist");
    }
    
    BellmanFordResult result = computeShortestPaths(startId);
    
    if (result.hasNegativeCycle) {
        std::cout << "[BELLMAN-FORD] Negative cycle detected - shortest paths undefined" << std::endl;
        return {};
    }
    
    if (result.distances.find(goalId) == result.distances.end() || 
        result.distances.at(goalId) == std::numeric_limits<double>::infinity()) {
        std::cout << "[BELLMAN-FORD] No path exists to goal node" << std::endl;
        return {};
    }
    
    return reconstructPath(result.predecessors, startId, goalId);
}

BellmanFord::BellmanFordResult BellmanFord::computeShortestPaths(int startId) {
    std::cout << "[BELLMAN-FORD] Computing shortest paths from node " << startId << std::endl;
    
    BellmanFordResult result;
    std::vector<int> nodeIds = graph->getAllNodeIds();
    
    if (nodeIds.empty()) {
        std::cout << "[BELLMAN-FORD] Empty graph provided" << std::endl;
        return result;
    }
    
    // Step 1: Initialize distances
    for (int nodeId : nodeIds) {
        result.distances[nodeId] = (nodeId == startId) ? 0.0 : std::numeric_limits<double>::infinity();
        result.predecessors[nodeId] = -1;
    }
    
    std::cout << "[BELLMAN-FORD] Initialized " << nodeIds.size() << " nodes" << std::endl;
    
    // Step 2: Relax edges repeatedly
    size_t nodeCount = nodeIds.size();
    bool hasImprovement = true;
    
    for (size_t iteration = 0; iteration < nodeCount - 1 && hasImprovement; ++iteration) {
        hasImprovement = false;
        std::cout << "[BELLMAN-FORD] Iteration " << (iteration + 1) << "/" << (nodeCount - 1) << std::endl;
        
        for (int nodeId : nodeIds) {
            if (result.distances[nodeId] == std::numeric_limits<double>::infinity()) {
                continue; // Skip unreachable nodes
            }
            
            const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
            for (const Edge& edge : edges) {
                int neighborId = edge.getToNode();
                double newDistance = result.distances[nodeId] + edge.getWeight();
                
                if (newDistance < result.distances[neighborId]) {
                    result.distances[neighborId] = newDistance;
                    result.predecessors[neighborId] = nodeId;
                    hasImprovement = true;
                    
                    std::cout << "[BELLMAN-FORD] Updated distance to node " << neighborId 
                              << ": " << newDistance << std::endl;
                }
            }
        }
        
        if (!hasImprovement) {
            std::cout << "[BELLMAN-FORD] Convergence achieved in iteration " << (iteration + 1) << std::endl;
            break;
        }
    }
    
    // Step 3: Check for negative cycles
    std::cout << "[BELLMAN-FORD] Checking for negative cycles..." << std::endl;
    result.hasNegativeCycle = false;
    
    for (int nodeId : nodeIds) {
        if (result.distances[nodeId] == std::numeric_limits<double>::infinity()) {
            continue;
        }
        
        const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
        for (const Edge& edge : edges) {
            int neighborId = edge.getToNode();
            double newDistance = result.distances[nodeId] + edge.getWeight();
            
            if (newDistance < result.distances[neighborId]) {
                result.hasNegativeCycle = true;
                result.negativeCycle = findNegativeCycle(result.predecessors);
                std::cout << "[BELLMAN-FORD] Negative cycle detected!" << std::endl;
                return result;
            }
        }
    }
    
    std::cout << "[BELLMAN-FORD] No negative cycles found" << std::endl;
    
    // Count reachable nodes
    size_t reachableCount = 0;
    for (const auto& pair : result.distances) {
        if (pair.second != std::numeric_limits<double>::infinity()) {
            reachableCount++;
        }
    }
    
    std::cout << "[BELLMAN-FORD] Computed shortest paths to " << reachableCount 
              << " out of " << nodeIds.size() << " nodes" << std::endl;
    
    return result;
}

double BellmanFord::getShortestDistance(int startId, int goalId) {
    BellmanFordResult result = computeShortestPaths(startId);
    
    if (result.hasNegativeCycle) {
        return -std::numeric_limits<double>::infinity(); // Indicate negative cycle
    }
    
    auto it = result.distances.find(goalId);
    return (it != result.distances.end()) ? it->second : std::numeric_limits<double>::infinity();
}

bool BellmanFord::detectNegativeCycle() {
    // Run algorithm from an arbitrary node to detect negative cycles
    std::vector<int> nodeIds = graph->getAllNodeIds();
    if (nodeIds.empty()) {
        return false;
    }
    
    BellmanFordResult result = computeShortestPaths(nodeIds[0]);
    return result.hasNegativeCycle;
}

std::vector<int> BellmanFord::getNegativeCycle() {
    std::vector<int> nodeIds = graph->getAllNodeIds();
    if (nodeIds.empty()) {
        return {};
    }
    
    BellmanFordResult result = computeShortestPaths(nodeIds[0]);
    return result.negativeCycle;
}

bool BellmanFord::canHandleNegativeWeights() const {
    return true; // Bellman-Ford algorithm specifically handles negative weights
}

void BellmanFord::printIterationDetails(bool enable) {
    iterationLogging = enable;
    if (enable) {
        std::cout << "[BELLMAN-FORD] Detailed iteration logging enabled" << std::endl;
    } else {
        std::cout << "[BELLMAN-FORD] Detailed iteration logging disabled" << std::endl;
    }
}

std::vector<int> BellmanFord::reconstructPath(const std::unordered_map<int, int>& predecessors, 
                                            int startId, int goalId) const {
    std::vector<int> path;
    int current = goalId;
    
    std::cout << "[BELLMAN-FORD] Reconstructing path from " << startId << " to " << goalId << std::endl;
    
    while (current != -1) {
        path.push_back(current);
        if (current == startId) {
            break;
        }
        
        auto it = predecessors.find(current);
        if (it != predecessors.end()) {
            current = it->second;
        } else {
            std::cout << "[BELLMAN-FORD] Path reconstruction failed - no predecessor for node " << current << std::endl;
            return {};
        }
    }
    
    if (path.empty() || path.back() != startId) {
        std::cout << "[BELLMAN-FORD] Invalid path reconstructed" << std::endl;
        return {};
    }
    
    std::reverse(path.begin(), path.end());
    
    std::cout << "[BELLMAN-FORD] Path reconstruction successful: ";
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << path[i];
        if (i < path.size() - 1) std::cout << " -> ";
    }
    std::cout << std::endl;
    
    return path;
}

std::vector<int> BellmanFord::findNegativeCycle(const std::unordered_map<int, int>& predecessors) const {
    std::cout << "[BELLMAN-FORD] Attempting to identify negative cycle" << std::endl;
    
    // Find a node that's part of a negative cycle
    std::vector<int> nodeIds = graph->getAllNodeIds();
    std::unordered_map<int, double> distances;
    
    // Initialize distances
    for (int nodeId : nodeIds) {
        distances[nodeId] = 0.0; // Start all nodes at 0 for cycle detection
    }
    
    // Run relaxation to find a node affected by negative cycle
    int affectedNode = -1;
    for (size_t iteration = 0; iteration < nodeIds.size(); ++iteration) {
        for (int nodeId : nodeIds) {
            const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
            for (const Edge& edge : edges) {
                int neighborId = edge.getToNode();
                double newDistance = distances[nodeId] + edge.getWeight();
                
                if (newDistance < distances[neighborId]) {
                    distances[neighborId] = newDistance;
                    affectedNode = neighborId;
                }
            }
        }
    }
    
    if (affectedNode == -1) {
        std::cout << "[BELLMAN-FORD] No negative cycle node identified" << std::endl;
        return {};
    }
    
    // Trace back from affected node to find the cycle
    std::vector<int> cycle;
    std::unordered_set<int> visited;
    int current = affectedNode;
    
    // Move back n steps to ensure we're in the cycle
    for (size_t i = 0; i < nodeIds.size(); ++i) {
        auto it = predecessors.find(current);
        if (it != predecessors.end()) {
            current = it->second;
        } else {
            break;
        }
    }
    
    // Now trace the cycle
    int cycleStart = current;
    do {
        cycle.push_back(current);
        auto it = predecessors.find(current);
        if (it != predecessors.end()) {
            current = it->second;
        } else {
            break;
        }
    } while (current != cycleStart && cycle.size() < nodeIds.size());
    
    if (current == cycleStart) {
        cycle.push_back(cycleStart); // Complete the cycle
        std::cout << "[BELLMAN-FORD] Negative cycle found with " << cycle.size() << " nodes" << std::endl;
    } else {
        std::cout << "[BELLMAN-FORD] Failed to trace complete negative cycle" << std::endl;
        cycle.clear();
    }
    
    return cycle;
}