#include "autonomous_navigator/AlgorithmSelector.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

class EnvironmentAnalyzer {
private:
    const Graph* graph;
    
public:
    explicit EnvironmentAnalyzer(const Graph* environment) : graph(environment) {}
    
    struct EnvironmentCharacteristics {
        size_t nodeCount;
        size_t edgeCount;
        double density;
        double averageDegree;
        double clusteringCoefficient;
        bool isGridLike;
        bool hasNegativeWeights;
        double maxEdgeWeight;
        double minEdgeWeight;
        double averageEdgeWeight;
    };
    
    EnvironmentCharacteristics analyzeEnvironment() const {
        EnvironmentCharacteristics characteristics;
        
        characteristics.nodeCount = graph->getNodeCount();
        characteristics.edgeCount = graph->getEdgeCount();
        
        if (characteristics.nodeCount > 0) {
            characteristics.density = static_cast<double>(characteristics.edgeCount) / 
                                    (characteristics.nodeCount * (characteristics.nodeCount - 1) / 2.0);
            characteristics.averageDegree = static_cast<double>(characteristics.edgeCount * 2) / characteristics.nodeCount;
        } else {
            characteristics.density = 0.0;
            characteristics.averageDegree = 0.0;
        }
        
        characteristics.clusteringCoefficient = calculateClusteringCoefficient();
        characteristics.isGridLike = detectGridStructure();
        
        analyzeEdgeWeights(characteristics);
        
        std::cout << "[ENV_SELECTION] Environment analysis completed:" << std::endl;
        std::cout << "[ENV_SELECTION]   Nodes: " << characteristics.nodeCount << std::endl;
        std::cout << "[ENV_SELECTION]   Edges: " << characteristics.edgeCount << std::endl;
        std::cout << "[ENV_SELECTION]   Density: " << characteristics.density << std::endl;
        std::cout << "[ENV_SELECTION]   Average degree: " << characteristics.averageDegree << std::endl;
        std::cout << "[ENV_SELECTION]   Grid-like: " << (characteristics.isGridLike ? "Yes" : "No") << std::endl;
        std::cout << "[ENV_SELECTION]   Has negative weights: " << (characteristics.hasNegativeWeights ? "Yes" : "No") << std::endl;
        
        return characteristics;
    }
    
    AlgorithmType recommendAlgorithmForEnvironment(const EnvironmentCharacteristics& env) const {
        std::cout << "[ENV_SELECTION] Selecting algorithm based on environment characteristics..." << std::endl;
        
        // Handle negative weights - only Bellman-Ford can handle them
        if (env.hasNegativeWeights) {
            std::cout << "[ENV_SELECTION] Negative weights detected, recommending Bellman-Ford" << std::endl;
            return AlgorithmType::BELLMAN_FORD;
        }
        
        // Very small graphs - BFS is sufficient for unweighted or simple cases
        if (env.nodeCount < 50 && env.averageEdgeWeight <= 1.1) {
            std::cout << "[ENV_SELECTION] Small unweighted graph, recommending BFS" << std::endl;
            return AlgorithmType::BFS;
        }
        
        // Grid-like structures with regular topology - Jump Point Search is optimal
        if (env.isGridLike && env.nodeCount > 100) {
            std::cout << "[ENV_SELECTION] Grid-like structure detected, recommending Jump Point Search" << std::endl;
            return AlgorithmType::JUMP_POINT_SEARCH;
        }
        
        // Dense graphs with many nodes - Dijkstra performs well
        if (env.density > 0.3 && env.nodeCount > 200) {
            std::cout << "[ENV_SELECTION] Dense graph detected, recommending Dijkstra" << std::endl;
            return AlgorithmType::DIJKSTRA;
        }
        
        // Sparse graphs with good heuristic potential - A* is ideal
        if (env.density < 0.1 && env.nodeCount > 50) {
            std::cout << "[ENV_SELECTION] Sparse graph with heuristic potential, recommending A*" << std::endl;
            return AlgorithmType::ASTAR;
        }
        
        // Medium-sized graphs with moderate density - A* is generally good
        if (env.nodeCount >= 50 && env.nodeCount <= 1000) {
            std::cout << "[ENV_SELECTION] Medium-sized graph, recommending A*" << std::endl;
            return AlgorithmType::ASTAR;
        }
        
        // Large sparse graphs - A* with good heuristics
        if (env.nodeCount > 1000 && env.density < 0.05) {
            std::cout << "[ENV_SELECTION] Large sparse graph, recommending A*" << std::endl;
            return AlgorithmType::ASTAR;
        }
        
        // Default case - A* is versatile for most scenarios
        std::cout << "[ENV_SELECTION] Default case, recommending A*" << std::endl;
        return AlgorithmType::ASTAR;
    }
    
    AlgorithmType selectForRealTimeConstraints(double maxExecutionTime, const EnvironmentCharacteristics& env) const {
        std::cout << "[ENV_SELECTION] Selecting algorithm for real-time constraint: " << maxExecutionTime << "s" << std::endl;
        
        // Very tight time constraints - use fastest algorithms
        if (maxExecutionTime < 0.001) { // 1ms
            if (env.averageEdgeWeight <= 1.1) {
                std::cout << "[ENV_SELECTION] Ultra-tight constraint, recommending BFS" << std::endl;
                return AlgorithmType::BFS;
            } else {
                std::cout << "[ENV_SELECTION] Ultra-tight constraint with weights, recommending A*" << std::endl;
                return AlgorithmType::ASTAR;
            }
        }
        
        // Tight constraints - prefer A* or JPS for efficiency
        if (maxExecutionTime < 0.01) { // 10ms
            if (env.isGridLike) {
                std::cout << "[ENV_SELECTION] Tight constraint on grid, recommending Jump Point Search" << std::endl;
                return AlgorithmType::JUMP_POINT_SEARCH;
            } else {
                std::cout << "[ENV_SELECTION] Tight constraint, recommending A*" << std::endl;
                return AlgorithmType::ASTAR;
            }
        }
        
        // Moderate constraints - can use more thorough algorithms
        if (maxExecutionTime < 0.1) { // 100ms
            if (env.hasNegativeWeights) {
                std::cout << "[ENV_SELECTION] Moderate constraint with negative weights, recommending Bellman-Ford" << std::endl;
                return AlgorithmType::BELLMAN_FORD;
            } else if (env.density > 0.3) {
                std::cout << "[ENV_SELECTION] Moderate constraint on dense graph, recommending Dijkstra" << std::endl;
                return AlgorithmType::DIJKSTRA;
            } else {
                std::cout << "[ENV_SELECTION] Moderate constraint, recommending A*" << std::endl;
                return AlgorithmType::ASTAR;
            }
        }
        
        // Relaxed constraints - can use any appropriate algorithm
        return recommendAlgorithmForEnvironment(env);
    }
    
    AlgorithmType selectForEnvironmentSize(size_t nodeCount, size_t edgeCount) const {
        double density = nodeCount > 1 ? static_cast<double>(edgeCount) / (nodeCount * (nodeCount - 1) / 2.0) : 0.0;
        
        std::cout << "[ENV_SELECTION] Selecting algorithm for size - Nodes: " << nodeCount 
                  << ", Edges: " << edgeCount << ", Density: " << density << std::endl;
        
        // Very small graphs
        if (nodeCount < 20) {
            std::cout << "[ENV_SELECTION] Very small graph, recommending BFS" << std::endl;
            return AlgorithmType::BFS;
        }
        
        // Small graphs
        if (nodeCount < 100) {
            if (density > 0.5) {
                std::cout << "[ENV_SELECTION] Small dense graph, recommending Dijkstra" << std::endl;
                return AlgorithmType::DIJKSTRA;
            } else {
                std::cout << "[ENV_SELECTION] Small sparse graph, recommending A*" << std::endl;
                return AlgorithmType::ASTAR;
            }
        }
        
        // Medium graphs
        if (nodeCount < 1000) {
            if (density > 0.2) {
                std::cout << "[ENV_SELECTION] Medium dense graph, recommending Dijkstra" << std::endl;
                return AlgorithmType::DIJKSTRA;
            } else {
                std::cout << "[ENV_SELECTION] Medium sparse graph, recommending A*" << std::endl;
                return AlgorithmType::ASTAR;
            }
        }
        
        // Large graphs - prefer A* for its heuristic advantage
        if (density < 0.05) {
            std::cout << "[ENV_SELECTION] Large sparse graph, recommending A*" << std::endl;
            return AlgorithmType::ASTAR;
        } else if (density > 0.1) {
            std::cout << "[ENV_SELECTION] Large dense graph, recommending Dijkstra" << std::endl;
            return AlgorithmType::DIJKSTRA;
        } else {
            std::cout << "[ENV_SELECTION] Large graph with moderate density, recommending A*" << std::endl;
            return AlgorithmType::ASTAR;
        }
    }
    
private:
    double calculateClusteringCoefficient() const {
        if (graph->getNodeCount() < 3) {
            return 0.0;
        }
        
        double totalCoefficient = 0.0;
        size_t validNodes = 0;
        
        for (int nodeId : graph->getAllNodeIds()) {
            std::vector<int> neighbors = graph->getNeighbors(nodeId);
            
            if (neighbors.size() < 2) {
                continue;
            }
            
            int triangles = 0;
            int possibleTriangles = neighbors.size() * (neighbors.size() - 1) / 2;
            
            for (size_t i = 0; i < neighbors.size(); ++i) {
                for (size_t j = i + 1; j < neighbors.size(); ++j) {
                    if (graph->hasEdge(neighbors[i], neighbors[j])) {
                        triangles++;
                    }
                }
            }
            
            if (possibleTriangles > 0) {
                totalCoefficient += static_cast<double>(triangles) / possibleTriangles;
                validNodes++;
            }
        }
        
        return validNodes > 0 ? totalCoefficient / validNodes : 0.0;
    }
    
    bool detectGridStructure() const {
        if (graph->getNodeCount() < 9) { // Minimum for a 3x3 grid
            return false;
        }
        
        // Check if most nodes have degree 2, 3, or 4 (typical for grid)
        std::vector<int> degreeDistribution(9, 0); // 0-8 degree
        
        for (int nodeId : graph->getAllNodeIds()) {
            int degree = graph->getNeighbors(nodeId).size();
            if (degree < 9) {
                degreeDistribution[degree]++;
            }
        }
        
        // In a grid, most nodes should have degree 2, 3, or 4
        int gridLikeNodes = degreeDistribution[2] + degreeDistribution[3] + degreeDistribution[4];
        double gridLikeRatio = static_cast<double>(gridLikeNodes) / graph->getNodeCount();
        
        return gridLikeRatio > 0.7; // 70% of nodes should have grid-like connectivity
    }
    
    void analyzeEdgeWeights(EnvironmentCharacteristics& characteristics) const {
        characteristics.hasNegativeWeights = false;
        characteristics.maxEdgeWeight = 0.0;
        characteristics.minEdgeWeight = std::numeric_limits<double>::max();
        characteristics.averageEdgeWeight = 0.0;
        
        double totalWeight = 0.0;
        size_t edgeCount = 0;
        
        for (int nodeId : graph->getAllNodeIds()) {
            const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
            
            for (const Edge& edge : edges) {
                double weight = edge.getWeight();
                
                if (weight < 0) {
                    characteristics.hasNegativeWeights = true;
                }
                
                characteristics.maxEdgeWeight = std::max(characteristics.maxEdgeWeight, weight);
                characteristics.minEdgeWeight = std::min(characteristics.minEdgeWeight, weight);
                
                totalWeight += weight;
                edgeCount++;
            }
        }
        
        if (edgeCount > 0) {
            characteristics.averageEdgeWeight = totalWeight / edgeCount;
        } else {
            characteristics.minEdgeWeight = 0.0;
        }
    }
};

// Implementation of environment-based selection methods for AlgorithmSelector
AlgorithmType AlgorithmSelector::analyzeEnvironmentCharacteristics() const {
    std::cout << "[ALGORITHM_SELECTOR] Analyzing environment characteristics for algorithm selection" << std::endl;
    
    if (!environment) {
        std::cout << "[ALGORITHM_SELECTOR] No environment provided, defaulting to A*" << std::endl;
        return AlgorithmType::ASTAR;
    }
    
    EnvironmentAnalyzer analyzer(environment);
    EnvironmentAnalyzer::EnvironmentCharacteristics characteristics = analyzer.analyzeEnvironment();
    
    return analyzer.recommendAlgorithmForEnvironment(characteristics);
}

AlgorithmType AlgorithmSelector::selectForEnvironmentSize(size_t nodeCount, size_t edgeCount) {
    std::cout << "[ALGORITHM_SELECTOR] Selecting algorithm based on environment size" << std::endl;
    
    if (!environment) {
        std::cout << "[ALGORITHM_SELECTOR] No environment available, using provided parameters" << std::endl;
        
        // Create a temporary analyzer for size-based selection
        if (nodeCount == 0) {
            return AlgorithmType::BFS; // Safe default for empty graphs
        }
        
        double density = nodeCount > 1 ? static_cast<double>(edgeCount) / (nodeCount * (nodeCount - 1) / 2.0) : 0.0;
        
        if (nodeCount < 50) {
            return density > 0.3 ? AlgorithmType::DIJKSTRA : AlgorithmType::BFS;
        } else if (nodeCount < 500) {
            return density > 0.2 ? AlgorithmType::DIJKSTRA : AlgorithmType::ASTAR;
        } else {
            return density > 0.1 ? AlgorithmType::DIJKSTRA : AlgorithmType::ASTAR;
        }
    }
    
    EnvironmentAnalyzer analyzer(environment);
    return analyzer.selectForEnvironmentSize(nodeCount, edgeCount);
}

AlgorithmType AlgorithmSelector::selectForRealTimeConstraints(double maxExecutionTime) {
    std::cout << "[ALGORITHM_SELECTOR] Selecting algorithm for real-time constraints: " 
              << maxExecutionTime << " seconds" << std::endl;
    
    if (!environment) {
        std::cout << "[ALGORITHM_SELECTOR] No environment available, using time-based heuristics" << std::endl;
        
        if (maxExecutionTime < 0.001) {
            return AlgorithmType::BFS;
        } else if (maxExecutionTime < 0.01) {
            return AlgorithmType::ASTAR;
        } else if (maxExecutionTime < 0.1) {
            return AlgorithmType::DIJKSTRA;
        } else {
            return AlgorithmType::ASTAR;
        }
    }
    
    EnvironmentAnalyzer analyzer(environment);
    EnvironmentAnalyzer::EnvironmentCharacteristics characteristics = analyzer.analyzeEnvironment();
    
    return analyzer.selectForRealTimeConstraints(maxExecutionTime, characteristics);
}