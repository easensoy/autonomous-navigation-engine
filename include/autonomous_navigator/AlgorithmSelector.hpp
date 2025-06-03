#pragma once
#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include "pathfinding_algorithms/BFS.hpp"
#include "pathfinding_algorithms/BellmanFord.hpp"
#include <memory>

enum class AlgorithmType {
    ASTAR,
    DIJKSTRA,
    BFS,
    DFS,
    BELLMAN_FORD,
    JUMP_POINT_SEARCH,
    AUTO_SELECT
};

struct AlgorithmPerformanceMetrics {
    double executionTime;
    size_t nodesExplored;
    double pathLength;
    size_t memoryUsage;
    double optimality;
};

class AlgorithmSelector {
private:
    const Graph* environment;
    std::unordered_map<AlgorithmType, std::unique_ptr<void>> algorithms;
    std::unordered_map<AlgorithmType, AlgorithmPerformanceMetrics> performanceHistory;
    
    AlgorithmType analyzeEnvironmentCharacteristics() const;
    AlgorithmType selectBasedOnPerformance(int startId, int goalId) const;

public:
    explicit AlgorithmSelector(const Graph* graph);
    
    AlgorithmType selectOptimalAlgorithm(int startId, int goalId);
    AlgorithmType selectForEnvironmentSize(size_t nodeCount, size_t edgeCount);
    AlgorithmType selectForRealTimeConstraints(double maxExecutionTime);
    
    std::vector<int> executeSelectedAlgorithm(AlgorithmType algorithm, int startId, int goalId);
    void benchmarkAlgorithms(int startId, int goalId);
    
    void setPerformancePriority(const std::string& priority);
    void updatePerformanceMetrics(AlgorithmType algorithm, const AlgorithmPerformanceMetrics& metrics);
    AlgorithmPerformanceMetrics getAlgorithmMetrics(AlgorithmType algorithm) const;
    void clearPerformanceHistory();
    
    void enableAdaptiveLearning(bool enable);
    AlgorithmType getRecommendedAlgorithm() const;
};