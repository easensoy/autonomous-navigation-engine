#pragma once
#include "../core/Graph.hpp"
#include <vector>

enum class OptimizationCriteria {
    MINIMIZE_DISTANCE,
    MINIMIZE_TIME,
    MINIMIZE_ENERGY,
    MINIMIZE_RISK,
    MULTI_OBJECTIVE
};

class PathOptimizer {
private:
    const Graph* environment;
    OptimizationCriteria currentCriteria;
    
    std::vector<int> optimizeForDistance(const std::vector<int>& originalPath);
    std::vector<int> optimizeForTime(const std::vector<int>& originalPath);
    std::vector<int> removeRedundantWaypoints(const std::vector<int>& path);

public:
    explicit PathOptimizer(const Graph* graph);
    
    std::vector<int> optimizePath(const std::vector<int>& originalPath);
    std::vector<int> optimizeWithCriteria(const std::vector<int>& originalPath, OptimizationCriteria criteria);
    std::vector<int> optimizeMultiObjective(const std::vector<int>& originalPath, const std::vector<double>& weights);
    
    void setOptimizationCriteria(OptimizationCriteria criteria);
    double calculateOptimizationImprovement(const std::vector<int>& originalPath, const std::vector<int>& optimizedPath);
    
    std::vector<int> shortcutPath(const std::vector<int>& path);
    std::vector<int> interpolateWaypoints(const std::vector<int>& sparsePath, int maxSegmentLength);
    
    void enableIterativeOptimization(bool enable);
    void setOptimizationIterations(int maxIterations);
    bool isOptimizationNecessary(const std::vector<int>& path) const;
};